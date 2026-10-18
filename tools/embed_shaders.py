#!/usr/bin/env python3
import sys
def arr(name, path):
    data = open(path, 'rb').read()
    assert len(data) % 4 == 0
    words = [int.from_bytes(data[i:i+4], 'little') for i in range(0, len(data), 4)]
    out = [f'static const uint32_t {name}[] = {{']
    for i in range(0, len(words), 8):
        out.append(','.join('0x%08x' % w for w in words[i:i+8]) + ',')
    out.append('};')
    out.append(f'static const size_t {name}_len = sizeof({name});')
    return '\n'.join(out)
body = '#pragma once\n#include <stdint.h>\n#include <stddef.h>\n'
body += arr('quad_vert_spv', sys.argv[1]) + '\n'
body += arr('quad_frag_spv', sys.argv[2]) + '\n'
open(sys.argv[3], 'w').write(body)
