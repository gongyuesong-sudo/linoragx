Name:           linoragx
Version:        0.1
Release:        1%{?dist}
Summary:        Neo Geo emulator (Wayland + Vulkan)
License:        GPL-3.0-or-later
URL:            https://github.com/gongyuesong-sudo/linoragx
Source0:        %{name}-%{version}.tar.gz

BuildRequires:  gcc, gcc-c++, zlib-devel, vulkan-headers, libwayland-headers
Requires:       vulkan-loader, libwayland-client, zlib, libstdc++

%description
linoragx is a Neo Geo MVS/AES emulator for Linux, a from-scratch modern
rewrite of the classic Windows emulator NeoRageX, using Wayland + Vulkan
rendering and an accurate YM2610 audio core. Requires a Neo Geo BIOS set
and game ROMs (not included).

%prep
%setup -q

%build
make -j%{?_smp_mflags} EXTRA_CFLAGS="-march=x86-64-v2"

%install
install -D -m755 build/linoragx %{buildroot}%{_bindir}/linoragx
install -D -m644 packaging/linoragx.desktop %{buildroot}%{_datadir}/applications/linoragx.desktop
install -D -m644 packaging/linoragx.png %{buildroot}%{_datadir}/icons/hicolor/256x256/apps/linoragx.png
install -D -m644 README.md %{buildroot}%{_docdir}/linoragx/README.md

%files
%doc README.md
%{_bindir}/linoragx
%{_datadir}/applications/linoragx.desktop
%{_datadir}/icons/hicolor/256x256/apps/linoragx.png

%changelog
* Tue Nov 03 2026 gongyuesong-sudo <gongyuesong-sudo@users.noreply.github.com> - 0.1-1
- First public release: 194/208 game titles with working audio,
  MAME-accurate YM2610/YM2151 golden audio core, Wayland+Vulkan renderer.
