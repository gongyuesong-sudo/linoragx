/* Vulkan renderer: Wayland window surface + offscreen readback mode.
 * Pipeline: 320x224 texture (per-frame upload) -> scaled quad, nearest filter. */
#include "linoragx.h"
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_wayland.h>
#include <wayland-client.h>
#include <wayland-client-protocol.h>
#include "xdg-shell-client-protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include "shaders_spv.h"

#define MAXF 4
struct vkrend {
    neogeo *ng;
    int scale, scanlines, headless;
    char err[256];

    VkInstance inst;
    VkSurfaceKHR surf;
    VkPhysicalDevice pdev;
    VkDevice dev;
    uint32_t qfi;
    VkQueue queue;

    VkSwapchainKHR sc;
    VkFormat scfmt;
    VkExtent2D scext;
    VkImage scimg[MAXF]; uint32_t nscimg;
    VkImageView scview[MAXF];
    VkRenderPass rp;
    VkFramebuffer fb[MAXF]; uint32_t nfb;
    VkDescriptorSetLayout dsl;
    VkPipelineLayout pl;
    VkPipeline pipe;
    VkCommandPool cmdpool;
    VkCommandBuffer cmdb[MAXF];
    VkSemaphore imgavail[MAXF], renderdone[MAXF];
    VkFence fences[MAXF];
    uint32_t fidx;
    int sc_valid;

    VkImage tex; VkDeviceMemory texmem; VkImageView texview; VkSampler samp;
    VkDescriptorPool dpool; VkDescriptorSet dset;
    VkBuffer stage; VkDeviceMemory stagemem;

    /* offscreen target */
    VkImage offimg; VkDeviceMemory offmem; VkImageView offview; VkFramebuffer offfb;
    VkBuffer readbuf; VkDeviceMemory readmem;

    /* wayland */
    struct wl_display *wdpy;
    struct wl_registry *wreg;
    struct wl_compositor *comp_;
    struct wl_surface *wsurf;
    struct xdg_wm_base *wmbase;
    struct xdg_surface *xsurf;
    struct xdg_toplevel *xtop;
    int configured; uint32_t cfgw, cfgh; int quit;
};

static void xdg_wmbase_ping(void *d, struct xdg_wm_base *b, uint32_t serial)
{ (void)d; xdg_wm_base_pong(b, serial); }
static const struct xdg_wm_base_listener wmbase_listener = { xdg_wmbase_ping };
static void xdg_surf_configure(void *d, struct xdg_surface *xs, uint32_t serial)
{
    struct vkrend *v = d;
    xdg_surface_ack_configure(xs, serial);   /* ack BEFORE any buffer commit */
    wl_display_flush(v->wdpy);
    fprintf(stderr, "[wl] xdg_surface configure serial=%u acked\n", serial);
    v->configured = 1;
}
static void xdg_top_configure(void *d, struct xdg_toplevel *t, int32_t w, int32_t h, struct wl_array *st)
{
    (void)t; (void)st;
    struct vkrend *v = d;
    uint32_t nw = w > 0 ? (uint32_t)w : NG_W * v->scale;
    uint32_t nh = h > 0 ? (uint32_t)h : NG_H * v->scale;
    if (v->cfgw && (nw != v->cfgw || nh != v->cfgh)) v->sc_valid = 0;   /* force swapchain rebuild */
    v->cfgw = nw; v->cfgh = nh;
    fprintf(stderr, "[wl] toplevel configure %ux%u\n", nw, nh);
    wl_surface_commit(v->wsurf);
}
static void xdg_top_close(void *d, struct xdg_toplevel *t) { (void)t; struct vkrend *v = d; v->quit = 1; }
static const struct xdg_surface_listener xsurf_listener = { xdg_surf_configure, NULL };
static const struct xdg_toplevel_listener xtop_listener = { xdg_top_configure, xdg_top_close, NULL, NULL };
static void registry_get(void *d, struct wl_registry *r, uint32_t name, const char *iface, uint32_t ver)
{
    struct vkrend *v = d;
    if (!strcmp(iface, "wl_compositor"))
        v->comp_ = wl_registry_bind(r, name, &wl_compositor_interface, ver < 4 ? ver : 4);
    else if (!strcmp(iface, "xdg_wm_base"))
        v->wmbase = wl_registry_bind(r, name, &xdg_wm_base_interface, ver < 3 ? ver : 3);
}
static void registry_rm(void *d, struct wl_registry *r, uint32_t n) { (void)d;(void)r;(void)n; }
static const struct wl_registry_listener wreg_listener = { registry_get, registry_rm };

static uint32_t find_mem(VkPhysicalDevice pd, uint32_t bits, VkMemoryPropertyFlags want)
{
    VkPhysicalDeviceMemoryProperties mp;
    vkGetPhysicalDeviceMemoryProperties(pd, &mp);
    for (uint32_t i = 0; i < mp.memoryTypeCount; i++)
        if ((bits & (1u<<i)) && (mp.memoryTypes[i].propertyFlags & want) == want) return i;
    return 0;
}

static void img_barrier(VkCommandBuffer cb, VkImage img, VkImageLayout from, VkImageLayout to,
                        VkAccessFlags srcA, VkAccessFlags dstA,
                        VkPipelineStageFlags srcS, VkPipelineStageFlags dstS)
{
    VkImageMemoryBarrier b = {VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    b.oldLayout = from; b.newLayout = to;
    b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    b.image = img;
    b.subresourceRange = (VkImageSubresourceRange){VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1};
    b.srcAccessMask = srcA; b.dstAccessMask = dstA;
    vkCmdPipelineBarrier(cb, srcS, dstS, 0, 0, NULL, 0, NULL, 1, &b);
}

static int mk_swapchain(struct vkrend *v, uint32_t want_w, uint32_t want_h)
{
    VkSurfaceCapabilitiesKHR caps;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(v->pdev, v->surf, &caps);
    v->scext = caps.currentExtent;
    if (v->scext.width == 0xFFFFFFFFu) {
        v->scext.width  = want_w  ? want_w  : NG_W * v->scale;
        v->scext.height = want_h  ? want_h  : NG_H * v->scale;
    }
    uint32_t nimg = caps.minImageCount + 1;
    if (caps.maxImageCount && nimg > caps.maxImageCount) nimg = caps.maxImageCount;
    if (nimg > MAXF) nimg = MAXF;
    VkSwapchainCreateInfoKHR sci = {VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    sci.surface = v->surf;
    sci.minImageCount = nimg;
    sci.imageFormat = v->scfmt;
    sci.imageColorSpace = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    sci.imageExtent = v->scext;
    sci.imageArrayLayers = 1;
    sci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    sci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    sci.preTransform = caps.currentTransform;
    sci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    sci.presentMode = VK_PRESENT_MODE_FIFO_KHR;
    sci.clipped = VK_TRUE;
    VkSwapchainKHR oldsc = v->sc;
    sci.oldSwapchain = oldsc;
    if (vkCreateSwapchainKHR(v->dev, &sci, NULL, &v->sc) < 0) return -1;
    if (oldsc) {                       /* release ALL in-flight work, then drop old objects */
        vkDeviceWaitIdle(v->dev);
        for (uint32_t i = 0; i < MAXF; i++) if (v->scview[i]) { vkDestroyImageView(v->dev, v->scview[i], NULL); v->scview[i] = VK_NULL_HANDLE; }
        vkDestroySwapchainKHR(v->dev, oldsc, NULL);
    }
    v->nscimg = MAXF;
    vkGetSwapchainImagesKHR(v->dev, v->sc, &v->nscimg, NULL);
    if (v->nscimg > MAXF) v->nscimg = MAXF;
    vkGetSwapchainImagesKHR(v->dev, v->sc, &v->nscimg, v->scimg);
    for (uint32_t i = 0; i < v->nscimg; i++) {
        VkImageViewCreateInfo vi = {VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        vi.image = v->scimg[i]; vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vi.format = v->scfmt;
        vi.subresourceRange = (VkImageSubresourceRange){VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1};
        if (vkCreateImageView(v->dev, &vi, NULL, &v->scview[i]) < 0) return -1;
    }
    return 0;
}

static int mk_renderpass(struct vkrend *v)
{
    VkAttachmentDescription color = {0};
    color.format = v->scfmt;
    color.samples = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color.finalLayout = v->headless ? VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL
                                    : VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    VkAttachmentReference cr = {0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription sp = {0};
    sp.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    sp.colorAttachmentCount = 1; sp.pColorAttachments = &cr;
    /* field order: srcSubpass,dstSubpass,srcStage,dstStage,srcAccess,dstAccess,flags */
    VkSubpassDependency deps[2] = {
        {VK_SUBPASS_EXTERNAL, 0, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, 0, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, 0},
        {0, VK_SUBPASS_EXTERNAL, VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, 0, 0},
    };
    VkRenderPassCreateInfo rpi = {VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    rpi.attachmentCount = 1; rpi.pAttachments = &color;
    rpi.subpassCount = 1; rpi.pSubpasses = &sp;
    rpi.dependencyCount = 2; rpi.pDependencies = deps;
    return vkCreateRenderPass(v->dev, &rpi, NULL, &v->rp);
}

static int mk_fbs(struct vkrend *v)
{
    for (uint32_t i = 0; i < v->nfb; i++) vkDestroyFramebuffer(v->dev, v->fb[i], NULL);
    v->nfb = v->nscimg;
    for (uint32_t i = 0; i < v->nfb; i++) {
        VkFramebufferCreateInfo fi = {VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        fi.renderPass = v->rp; fi.attachmentCount = 1; fi.pAttachments = &v->scview[i];
        fi.width = v->scext.width; fi.height = v->scext.height; fi.layers = 1;
        if (vkCreateFramebuffer(v->dev, &fi, NULL, &v->fb[i]) < 0) return -1;
    }
    return 0;
}

vkrend *vk_init(neogeo *ng, const char *wayland_display, int scale, int scanlines, char *err, size_t errsz)
{
    struct vkrend *v = calloc(1, sizeof *v);
    v->ng = ng; v->scale = scale < 1 ? 1 : scale; v->scanlines = scanlines;
    err[0] = 0;

    if (wayland_display) {
        v->wdpy = wl_display_connect(wayland_display);
        if (!v->wdpy) { snprintf(err, errsz, "wayland connect failed"); free(v); return NULL; }
        v->wreg = wl_display_get_registry(v->wdpy);
        wl_registry_add_listener(v->wreg, &wreg_listener, v);
        wl_display_roundtrip(v->wdpy);
        if (!v->comp_) { snprintf(err, errsz, "no wl_compositor"); goto fail; }
        if (!v->wmbase) { snprintf(err, errsz, "no xdg_wm_base"); goto fail; }
        v->wsurf = wl_compositor_create_surface(v->comp_);
        v->xsurf = xdg_wm_base_get_xdg_surface(v->wmbase, v->wsurf);
        xdg_surface_add_listener(v->xsurf, &xsurf_listener, v);
        v->xtop = xdg_surface_get_toplevel(v->xsurf);
        xdg_toplevel_add_listener(v->xtop, &xtop_listener, v);
        xdg_toplevel_set_app_id(v->xtop, "com.linoragx.emulator");
        xdg_toplevel_set_title(v->xtop, "linoragx - Neo Geo");
        wl_surface_commit(v->wsurf);              /* empty commit: request configure */
        while (!v->configured || !v->cfgw) {      /* NEVER present a buffer before this */
            if (wl_display_roundtrip(v->wdpy) < 0) { snprintf(err, errsz, "xdg roundtrip"); goto fail; }
            if (!v->configured || !v->cfgw) usleep(20000);
        }
    } else v->headless = 1;

    const char *iext[8]; int ni = 0;
    iext[ni++] = VK_KHR_SURFACE_EXTENSION_NAME;
    if (!v->headless) iext[ni++] = VK_KHR_WAYLAND_SURFACE_EXTENSION_NAME;
    VkApplicationInfo app = {VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.apiVersion = VK_API_VERSION_1_1; app.pApplicationName = "linoragx";
    VkInstanceCreateInfo ici = {VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    ici.pApplicationInfo = &app; ici.enabledExtensionCount = ni; ici.ppEnabledExtensionNames = iext;
    if (vkCreateInstance(&ici, NULL, &v->inst) < 0) { snprintf(err, errsz, "vkCreateInstance (no ICD?)"); goto fail; }

    if (!v->headless) {
        VkWaylandSurfaceCreateInfoKHR wsci = {VK_STRUCTURE_TYPE_WAYLAND_SURFACE_CREATE_INFO_KHR};
        wsci.display = v->wdpy; wsci.surface = v->wsurf;
        if (vkCreateWaylandSurfaceKHR(v->inst, &wsci, NULL, &v->surf) < 0) { snprintf(err, errsz, "wayland surface"); goto fail; }
    }

    uint32_t npd = 0; vkEnumeratePhysicalDevices(v->inst, &npd, NULL);
    if (!npd) { snprintf(err, errsz, "no vulkan devices"); goto fail; }
    VkPhysicalDevice pds[16]; if (npd > 16) npd = 16;
    vkEnumeratePhysicalDevices(v->inst, &npd, pds);
    v->pdev = pds[0];
    for (uint32_t i = 0; i < npd; i++) {
        VkPhysicalDeviceProperties pp; vkGetPhysicalDeviceProperties(pds[i], &pp);
        if (pp.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) { v->pdev = pds[i]; break; }
    }

    uint32_t nq = 0; vkGetPhysicalDeviceQueueFamilyProperties(v->pdev, &nq, NULL);
    VkQueueFamilyProperties qfp[16]; if (nq > 16) nq = 16;
    vkGetPhysicalDeviceQueueFamilyProperties(v->pdev, &nq, qfp);
    v->qfi = UINT32_MAX;
    for (uint32_t i = 0; i < nq; i++) {
        if (!(qfp[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)) continue;
        if (v->surf) {
            VkBool32 ok; vkGetPhysicalDeviceSurfaceSupportKHR(v->pdev, i, v->surf, &ok);
            if (!ok) continue;
        }
        v->qfi = i; break;
    }
    if (v->qfi == UINT32_MAX) { snprintf(err, errsz, "no graphics queue"); goto fail; }

    float prio = 1;
    VkDeviceQueueCreateInfo qci = {VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    qci.queueFamilyIndex = v->qfi; qci.queueCount = 1; qci.pQueuePriorities = &prio;
    const char *dext[] = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };
    VkDeviceCreateInfo dci = {VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    dci.queueCreateInfoCount = 1; dci.pQueueCreateInfos = &qci;
    dci.enabledExtensionCount = 1; dci.ppEnabledExtensionNames = dext;
    if (vkCreateDevice(v->pdev, &dci, NULL, &v->dev) < 0) { snprintf(err, errsz, "vkCreateDevice"); goto fail; }
    vkGetDeviceQueue(v->dev, v->qfi, 0, &v->queue);

    /* staging buffer */
    {
        VkBufferCreateInfo bi = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bi.size = NG_W*NG_H*4; bi.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        vkCreateBuffer(v->dev, &bi, NULL, &v->stage);
        VkMemoryRequirements req; vkGetBufferMemoryRequirements(v->dev, v->stage, &req);
        VkMemoryAllocateInfo ai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.allocationSize = req.size;
        ai.memoryTypeIndex = find_mem(v->pdev, req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkAllocateMemory(v->dev, &ai, NULL, &v->stagemem);
        vkBindBufferMemory(v->dev, v->stage, v->stagemem, 0);
    }
    /* texture */
    {
        VkImageCreateInfo ti = {VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ti.imageType = VK_IMAGE_TYPE_2D; ti.format = VK_FORMAT_R8G8B8A8_UNORM;
        ti.extent = (VkExtent3D){NG_W, NG_H, 1};
        ti.mipLevels = ti.arrayLayers = 1;
        ti.samples = VK_SAMPLE_COUNT_1_BIT; ti.tiling = VK_IMAGE_TILING_OPTIMAL;
        ti.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ti.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        vkCreateImage(v->dev, &ti, NULL, &v->tex);
        VkMemoryRequirements req; vkGetImageMemoryRequirements(v->dev, v->tex, &req);
        VkMemoryAllocateInfo ai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.allocationSize = req.size;
        ai.memoryTypeIndex = find_mem(v->pdev, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkAllocateMemory(v->dev, &ai, NULL, &v->texmem);
        vkBindImageMemory(v->dev, v->tex, v->texmem, 0);
        VkImageViewCreateInfo vi = {VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        vi.image = v->tex; vi.viewType = VK_IMAGE_VIEW_TYPE_2D; vi.format = VK_FORMAT_R8G8B8A8_UNORM;
        vi.subresourceRange = (VkImageSubresourceRange){VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1};
        vkCreateImageView(v->dev, &vi, NULL, &v->texview);
        VkSamplerCreateInfo si = {VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
        si.magFilter = si.minFilter = VK_FILTER_NEAREST;
        si.addressModeU = si.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        vkCreateSampler(v->dev, &si, NULL, &v->samp);
    }
    /* descriptor layout + pool + set */
    {
        VkDescriptorSetLayoutBinding b0 = {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, NULL};
        VkDescriptorSetLayoutCreateInfo dli = {VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
        dli.bindingCount = 1; dli.pBindings = &b0;
        vkCreateDescriptorSetLayout(v->dev, &dli, NULL, &v->dsl);
        VkDescriptorPoolSize ps = {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 4};
        VkDescriptorPoolCreateInfo dpi = {VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
        dpi.maxSets = 4; dpi.poolSizeCount = 1; dpi.pPoolSizes = &ps;
        vkCreateDescriptorPool(v->dev, &dpi, NULL, &v->dpool);
        VkDescriptorSetAllocateInfo dai = {VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
        dai.descriptorPool = v->dpool; dai.descriptorSetCount = 1; dai.pSetLayouts = &v->dsl;
        vkAllocateDescriptorSets(v->dev, &dai, &v->dset);
        VkDescriptorImageInfo di = {v->samp, v->texview, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
        VkWriteDescriptorSet w = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
        w.dstSet = v->dset; w.dstBinding = 0; w.descriptorCount = 1;
        w.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        w.pImageInfo = &di;
        vkUpdateDescriptorSets(v->dev, 1, &w, 0, NULL);
    }
    /* swapchain format */
    if (!v->headless) {
        uint32_t nf = 0; vkGetPhysicalDeviceSurfaceFormatsKHR(v->pdev, v->surf, &nf, NULL);
        VkSurfaceFormatKHR sf[16]; if (nf > 16) nf = 16;
        vkGetPhysicalDeviceSurfaceFormatsKHR(v->pdev, v->surf, &nf, sf);
        v->scfmt = sf[0].format;
        for (uint32_t i = 0; i < nf; i++)
            if (sf[i].format == VK_FORMAT_B8G8R8A8_UNORM || sf[i].format == VK_FORMAT_R8G8B8A8_UNORM) { v->scfmt = sf[i].format; break; }
        if (mk_swapchain(v, v->cfgw, v->cfgh) < 0) { snprintf(err, errsz, "swapchain"); goto fail; }
    } else {
        v->scfmt = VK_FORMAT_R8G8B8A8_UNORM;
        v->scext = (VkExtent2D){NG_W*2, NG_H*2};
        /* offscreen color image */
        VkImageCreateInfo ti = {VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ti.imageType = VK_IMAGE_TYPE_2D; ti.format = v->scfmt;
        ti.extent = (VkExtent3D){v->scext.width, v->scext.height, 1};
        ti.mipLevels = ti.arrayLayers = 1;
        ti.samples = VK_SAMPLE_COUNT_1_BIT; ti.tiling = VK_IMAGE_TILING_OPTIMAL;
        ti.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
        ti.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        vkCreateImage(v->dev, &ti, NULL, &v->offimg);
        VkMemoryRequirements req; vkGetImageMemoryRequirements(v->dev, v->offimg, &req);
        VkMemoryAllocateInfo ai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.allocationSize = req.size;
        ai.memoryTypeIndex = find_mem(v->pdev, req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkAllocateMemory(v->dev, &ai, NULL, &v->offmem);
        vkBindImageMemory(v->dev, v->offimg, v->offmem, 0);
        VkImageViewCreateInfo vi = {VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        vi.image = v->offimg; vi.viewType = VK_IMAGE_VIEW_TYPE_2D; vi.format = v->scfmt;
        vi.subresourceRange = (VkImageSubresourceRange){VK_IMAGE_ASPECT_COLOR_BIT,0,1,0,1};
        vkCreateImageView(v->dev, &vi, NULL, &v->offview);
        /* readback buffer */
        VkBufferCreateInfo bi = {VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bi.size = (VkDeviceSize)v->scext.width * v->scext.height * 4;
        bi.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        vkCreateBuffer(v->dev, &bi, NULL, &v->readbuf);
        vkGetBufferMemoryRequirements(v->dev, v->readbuf, &req);
        VkMemoryAllocateInfo rai = {VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        rai.allocationSize = req.size;
        rai.memoryTypeIndex = find_mem(v->pdev, req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT|VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        vkAllocateMemory(v->dev, &rai, NULL, &v->readmem);
        vkBindBufferMemory(v->dev, v->readbuf, v->readmem, 0);
    }
    if (mk_renderpass(v) < 0) { snprintf(err, errsz, "renderpass"); goto fail; }
    if (!v->headless) {
        v->nfb = v->nscimg;
        if (mk_fbs(v) < 0) { snprintf(err, errsz, "framebuffers"); goto fail; }
    } else {
        VkFramebufferCreateInfo fi = {VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        fi.renderPass = v->rp; fi.attachmentCount = 1; fi.pAttachments = &v->offview;
        fi.width = v->scext.width; fi.height = v->scext.height; fi.layers = 1;
        vkCreateFramebuffer(v->dev, &fi, NULL, &v->offfb);
        v->fb[0] = v->offfb; v->nfb = 1;
    }
    /* pipeline */
    {
        VkPushConstantRange pcr = {VK_SHADER_STAGE_FRAGMENT_BIT, 0, 4};
        VkPipelineLayoutCreateInfo pli = {VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
        pli.setLayoutCount = 1; pli.pSetLayouts = &v->dsl;
        pli.pushConstantRangeCount = 1; pli.pPushConstantRanges = &pcr;
        vkCreatePipelineLayout(v->dev, &pli, NULL, &v->pl);
        VkShaderModuleCreateInfo vsi = {VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        vsi.codeSize = quad_vert_spv_len; vsi.pCode = quad_vert_spv;
        VkShaderModule vs, fs;
        vkCreateShaderModule(v->dev, &vsi, NULL, &vs);
        VkShaderModuleCreateInfo fsi = {VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
        fsi.codeSize = quad_frag_spv_len; fsi.pCode = quad_frag_spv;
        vkCreateShaderModule(v->dev, &fsi, NULL, &fs);
        VkPipelineShaderStageCreateInfo stg[2] = {{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO},{VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO}};
        stg[0].stage = VK_SHADER_STAGE_VERTEX_BIT; stg[0].module = vs; stg[0].pName = "main";
        stg[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT; stg[1].module = fs; stg[1].pName = "main";
        VkPipelineVertexInputStateCreateInfo vin = {VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
        VkPipelineInputAssemblyStateCreateInfo ias = {VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
        ias.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
        VkPipelineViewportStateCreateInfo vps = {VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
        vps.viewportCount = vps.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo rs = {VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
        rs.lineWidth = 1;
        VkPipelineMultisampleStateCreateInfo ms = {VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineColorBlendAttachmentState cba = {0};
        cba.colorWriteMask = 0xF;
        VkPipelineColorBlendStateCreateInfo cb = {VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
        cb.attachmentCount = 1; cb.pAttachments = &cba;
        VkDynamicState dyn[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dyi = {VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
        dyi.dynamicStateCount = 2; dyi.pDynamicStates = dyn;
        VkGraphicsPipelineCreateInfo gpi = {VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
        gpi.stageCount = 2; gpi.pStages = stg;
        gpi.pVertexInputState = &vin; gpi.pInputAssemblyState = &ias;
        gpi.pViewportState = &vps; gpi.pRasterizationState = &rs;
        gpi.pMultisampleState = &ms; gpi.pColorBlendState = &cb;
        gpi.pDynamicState = &dyi;
        gpi.layout = v->pl; gpi.renderPass = v->rp;
        VkResult mr1 = vkCreateShaderModule(v->dev, &vsi, NULL, &vs);
        VkResult mr2 = vkCreateShaderModule(v->dev, &fsi, NULL, &fs);
        VkResult pr_ = vkCreateGraphicsPipelines(v->dev, NULL, 1, &gpi, NULL, &v->pipe);
        if (mr1) fprintf(stderr, "[vk] vert module %d (len %zu)\n", mr1, vsi.codeSize);
        if (mr2) fprintf(stderr, "[vk] frag module %d (len %zu)\n", mr2, fsi.codeSize);
        if (pr_) fprintf(stderr, "[vk] pipeline %d\n", pr_);
        if (mr1 || mr2 || pr_) { snprintf(err, errsz, "pipeline"); goto fail; }
        vkDestroyShaderModule(v->dev, vs, NULL);
        vkDestroyShaderModule(v->dev, fs, NULL);
    }
    /* cmd pool + buffers + sync */
    {
        VkCommandPoolCreateInfo cpi = {VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        cpi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        cpi.queueFamilyIndex = v->qfi;
        vkCreateCommandPool(v->dev, &cpi, NULL, &v->cmdpool);
        VkCommandBufferAllocateInfo cai = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cai.commandPool = v->cmdpool; cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cai.commandBufferCount = MAXF;
        vkAllocateCommandBuffers(v->dev, &cai, v->cmdb);
        for (int i = 0; i < MAXF; i++) {
            VkSemaphoreCreateInfo sei = {VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
            vkCreateSemaphore(v->dev, &sei, NULL, &v->imgavail[i]);
            vkCreateSemaphore(v->dev, &sei, NULL, &v->renderdone[i]);
            VkFenceCreateInfo fei = {VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
            fei.flags = VK_FENCE_CREATE_SIGNALED_BIT;
            vkCreateFence(v->dev, &fei, NULL, &v->fences[i]);
        }
    }
    v->sc_valid = !v->headless;
    if (!v->headless) wl_surface_commit(v->wsurf);
    return v;

fail:
    if (!err[0]) snprintf(err, errsz, "vk init failed");
    if (err && errsz) { char t[256]; snprintf(t, sizeof t, "%s", err[0]?err:"fail"); snprintf(err, errsz, "%s", t); }
    vk_destroy(v);
    return NULL;
}

void vk_destroy(vkrend *vv)
{
    struct vkrend *v = vv;
    if (!v) return;
    if (v->dev) vkDeviceWaitIdle(v->dev);
    if (v->dev) {
        for (int i = 0; i < MAXF; i++) {
            if (v->imgavail[i]) vkDestroySemaphore(v->dev, v->imgavail[i], NULL);
            if (v->renderdone[i]) vkDestroySemaphore(v->dev, v->renderdone[i], NULL);
            if (v->fences[i]) vkDestroyFence(v->dev, v->fences[i], NULL);
        }
        if (v->cmdpool) vkDestroyCommandPool(v->dev, v->cmdpool, NULL);
        if (v->pipe) vkDestroyPipeline(v->dev, v->pipe, NULL);
        if (v->pl) vkDestroyPipelineLayout(v->dev, v->pl, NULL);
        if (v->dpool) vkDestroyDescriptorPool(v->dev, v->dpool, NULL);
        if (v->dsl) vkDestroyDescriptorSetLayout(v->dev, v->dsl, NULL);
        if (v->rp) vkDestroyRenderPass(v->dev, v->rp, NULL);
        for (uint32_t i = 0; i < v->nfb; i++) if (v->fb[i]) vkDestroyFramebuffer(v->dev, v->fb[i], NULL);
        for (uint32_t i = 0; i < v->nscimg; i++) if (v->scview[i]) vkDestroyImageView(v->dev, v->scview[i], NULL);
        if (v->sc) vkDestroySwapchainKHR(v->dev, v->sc, NULL);
        if (v->samp) vkDestroySampler(v->dev, v->samp, NULL);
        if (v->texview) vkDestroyImageView(v->dev, v->texview, NULL);
        if (v->tex) vkDestroyImage(v->dev, v->tex, NULL);
        if (v->texmem) vkFreeMemory(v->dev, v->texmem, NULL);
        if (v->stage) vkDestroyBuffer(v->dev, v->stage, NULL);
        if (v->stagemem) vkFreeMemory(v->dev, v->stagemem, NULL);
        if (v->offview) vkDestroyImageView(v->dev, v->offview, NULL);
        if (v->offimg) vkDestroyImage(v->dev, v->offimg, NULL);
        if (v->offmem) vkFreeMemory(v->dev, v->offmem, NULL);
        if (v->readbuf) vkDestroyBuffer(v->dev, v->readbuf, NULL);
        if (v->readmem) vkFreeMemory(v->dev, v->readmem, NULL);
        vkDestroyDevice(v->dev, NULL);
    }
    if (v->surf) vkDestroySurfaceKHR(v->inst, v->surf, NULL);
    if (v->inst) vkDestroyInstance(v->inst, NULL);
    if (v->wsurf) wl_surface_destroy(v->wsurf);
    if (v->wreg) wl_registry_destroy(v->wreg);
    if (v->wdpy) wl_display_disconnect(v->wdpy);
    free(v);
}

/* upload fb into tex + record render into cmdb[i] targeting view */
static int record_frame(struct vkrend *v, const uint32_t *fb, VkCommandBuffer cb, VkFramebuffer fbo)
{
    void *map; vkMapMemory(v->dev, v->stagemem, 0, NG_W*NG_H*4, 0, &map);
    memcpy(map, fb, NG_W*NG_H*4);
    vkUnmapMemory(v->dev, v->stagemem);

    VkCommandBufferBeginInfo bi = {VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    if (vkBeginCommandBuffer(cb, &bi) < 0) return -1;

    img_barrier(cb, v->tex, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                0, VK_ACCESS_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
    VkBufferImageCopy cp = {0};
    cp.imageSubresource = (VkImageSubresourceLayers){VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};
    cp.imageExtent = (VkExtent3D){NG_W, NG_H, 1};
    vkCmdCopyBufferToImage(cb, v->stage, v->tex, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &cp);
    img_barrier(cb, v->tex, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);

    VkClearValue cv = {{{0,0,0,1}}};
    VkRenderPassBeginInfo rbi = {VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    rbi.renderPass = v->rp; rbi.framebuffer = fbo;
    rbi.renderArea = (VkRect2D){{0,0}, v->scext};
    rbi.clearValueCount = 1; rbi.pClearValues = &cv;
    vkCmdBeginRenderPass(cb, &rbi, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, v->pipe);
    VkViewport vp = {0, 0, (float)v->scext.width, (float)v->scext.height, 0, 1};
    vkCmdSetViewport(cb, 0, 1, &vp);
    VkRect2D sc = {{0,0}, v->scext};
    vkCmdSetScissor(cb, 0, 1, &sc);
    vkCmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, v->pl, 0, 1, &v->dset, 0, NULL);
    int sl = v->scanlines ? 1 : 0;
    vkCmdPushConstants(cb, v->pl, VK_SHADER_STAGE_FRAGMENT_BIT, 0, 4, &sl);
    vkCmdDraw(cb, 4, 1, 0, 0);
    vkCmdEndRenderPass(cb);
    if (v->headless) {
        img_barrier(cb, v->offimg, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR == 0 ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL : VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                    VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT,
                    VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
        VkBufferImageCopy rcp = {0};
        rcp.imageSubresource = (VkImageSubresourceLayers){VK_IMAGE_ASPECT_COLOR_BIT,0,0,1};
        rcp.imageExtent = (VkExtent3D){v->scext.width, v->scext.height, 1};
        vkCmdCopyImageToBuffer(cb, v->offimg, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, v->readbuf, 1, &rcp);
    }
    return vkEndCommandBuffer(cb) < 0 ? -1 : 0;
}

static void wl_pump_events(struct vkrend *v)
{
    wl_display_flush(v->wdpy);
    struct pollfd pfd = { wl_display_get_fd(v->wdpy), POLLIN, 0 };
    if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN))
        wl_display_dispatch(v->wdpy);          /* read socket + dispatch */
    wl_display_dispatch_pending(v->wdpy);      /* drain queued */
}

int vk_pump(vkrend *v)
{
    if (v->headless) return 0;
    wl_pump_events(v);
    if (wl_display_get_error(v->wdpy) || v->quit) return 1;   /* compositor closed us */
    return 0;
}

static void wait_fence_wl(struct vkrend *v, VkFence f)
{
    /* keep the Wayland loop alive (xdg pings) while the GPU catches up */
    while (vkWaitForFences(v->dev, 1, &f, VK_TRUE, 8*1000*1000) == VK_TIMEOUT) {
        wl_pump_events(v);
        if (wl_display_get_error(v->wdpy)) return;
    }
}

void vk_present(vkrend *v, const uint32_t *fb)
{
    if (v->headless || !v->configured) return;
    if (!v->sc_valid || v->scext.width < 32u) {
        if (mk_swapchain(v, v->cfgw, v->cfgh) == 0) { v->nfb = v->nscimg; mk_fbs(v); v->sc_valid = 1; }
    }
    uint32_t i = v->fidx % MAXF; v->fidx++;
    wait_fence_wl(v, v->fences[i]);
    uint32_t img = 0;
    VkResult ar;
    for (;;) {
        ar = vkAcquireNextImageKHR(v->dev, v->sc, 500*1000*1000, v->imgavail[i], NULL, &img);
        if (ar != VK_TIMEOUT) break;
        wl_pump_events(v);                       /* stay responsive while compositor is slow */
    }
    if (ar == VK_ERROR_OUT_OF_DATE_KHR) {
        if (mk_swapchain(v, v->cfgw, v->cfgh) < 0) return;
        v->nfb = v->nscimg; mk_fbs(v);
        return;
    }
    if (ar < 0 && ar != VK_SUBOPTIMAL_KHR) return;
    if (img >= v->nfb) return;
    vkResetFences(v->dev, 1, &v->fences[i]);
    if (record_frame(v, fb, v->cmdb[i], v->fb[img]) < 0) return;
    VkPipelineStageFlags waitst = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo si = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.waitSemaphoreCount = 1; si.pWaitSemaphores = &v->imgavail[i];
    si.pWaitDstStageMask = &waitst;
    si.commandBufferCount = 1; si.pCommandBuffers = &v->cmdb[i];
    si.signalSemaphoreCount = 1; si.pSignalSemaphores = &v->renderdone[i];
    if (vkQueueSubmit(v->queue, 1, &si, v->fences[i]) < 0) return;
    VkPresentInfoKHR pi = {VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    pi.waitSemaphoreCount = 1; pi.pWaitSemaphores = &v->renderdone[i];
    pi.swapchainCount = 1; pi.pSwapchains = &v->sc; pi.pImageIndices = &img;
    VkResult pr = vkQueuePresentKHR(v->queue, &pi);
    if (pr == VK_ERROR_OUT_OF_DATE_KHR || pr == VK_SUBOPTIMAL_KHR) {
        mk_swapchain(v, v->cfgw, v->cfgh); v->nfb = v->nscimg; mk_fbs(v);
    }
}

static int write_bmp(const char *path, uint32_t w, uint32_t h, const uint8_t *rgba)
{
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    uint32_t row = ((w*3+3)/4)*4;
    uint32_t fsz = 54 + row*h;
    uint8_t hdr[54] = {'B','A'};
    hdr[2]=fsz; hdr[2]>>=0; *(uint32_t*)(hdr+2)=fsz; *(uint32_t*)(hdr+10)=54;
    *(uint32_t*)(hdr+14)=40; *(int32_t*)(hdr+18)=w; *(int32_t*)(hdr+22)=h;
    *(uint16_t*)(hdr+26)=1; *(uint16_t*)(hdr+28)=24; *(uint32_t*)(hdr+34)=row*h;
    fwrite(hdr,1,54,f);
    uint8_t *line = malloc(row);
    for (uint32_t y = 0; y < h; y++) {
        const uint8_t *s = rgba + (h-1-y)*w*4;
        for (uint32_t x = 0; x < w; x++) { line[x*3+0]=s[x*4+2]; line[x*3+1]=s[x*4+1]; line[x*3+2]=s[x*4+0]; }
        for (uint32_t x = w*3; x < row; x++) line[x] = 0;
        fwrite(line,1,row,f);
    }
    free(line); fclose(f);
    return 0;
}

int vk_offscreen_dump(vkrend *v, const uint32_t *fb, const char *bmp_path)
{
    if (!v->headless) return -1;
    uint32_t i = 0;
    vkWaitForFences(v->dev, 1, &v->fences[i], VK_TRUE, UINT64_MAX);
    vkResetFences(v->dev, 1, &v->fences[i]);
    if (record_frame(v, fb, v->cmdb[i], v->offfb) < 0) return -1;
    VkSubmitInfo si = {VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1; si.pCommandBuffers = &v->cmdb[i];
    if (vkQueueSubmit(v->queue, 1, &si, v->fences[i]) < 0) return -1;
    vkWaitForFences(v->dev, 1, &v->fences[i], VK_TRUE, UINT64_MAX);
    void *map;
    vkMapMemory(v->dev, v->readmem, 0, (VkDeviceSize)v->scext.width*v->scext.height*4, 0, &map);
    int r = write_bmp(bmp_path, v->scext.width, v->scext.height, map);
    vkUnmapMemory(v->dev, v->readmem);
    return r;
}
