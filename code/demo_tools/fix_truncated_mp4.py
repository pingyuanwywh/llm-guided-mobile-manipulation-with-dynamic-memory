#!/usr/bin/env python3
# encoding: utf-8
"""修复被硬掉电截断的 H.264 mp4(缺 moov atom)。2026-08-27。

record_cam.py 用 cv2.VideoWriter(avc1) 写 mp4, SPS/PPS 放在文件尾的 moov/avcC 里。
掉电 => moov 没写成 => 整个文件打不开, 但 mdat 里的 NAL 单元(AVCC 4字节长度前缀)是完整的。
修法: 从一个**同编码器录的好文件**取 SPS/PPS -> 把 mdat 转成 Annex-B 并在每个 IDR 前插入
      -> ffmpeg 按固定帧率重新封装。

  python3 fix_truncated_mp4.py 坏文件.mp4 参考好文件.mp4 输出.mp4 [帧率]
"""
import os, re, struct, subprocess, sys


def find_box(buf, want, start=0, end=None):
    """在 buf[start:end] 里逐个 box 走, 返回 (payload_start, payload_end)。"""
    end = len(buf) if end is None else end
    p = start
    while p + 8 <= end:
        size = struct.unpack('>I', buf[p:p + 4])[0]
        typ = buf[p + 4:p + 8]
        body = p + 8
        if size == 1:
            size = struct.unpack('>Q', buf[p + 8:p + 16])[0]; body = p + 16
        elif size == 0:
            size = end - p
        if typ == want:
            return body, p + size
        if typ in (b'moov', b'trak', b'mdia', b'minf', b'stbl', b'stsd', b'avc1'):
            r = find_box(buf, want, body + (78 if typ == b'avc1' else 0), p + size)
            if r:
                return r
        p += size
    return None


def get_sps_pps(ref_path):
    """让 ffmpeg 把参考文件转成 Annex-B, 再从里面捞 SPS(7)/PPS(8) —— 比手撕 avc1/avcC 偏移可靠。"""
    tmp = '/tmp/_ref_annexb.h264'
    subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', '-i', ref_path, '-c', 'copy',
                    '-bsf:v', 'h264_mp4toannexb', '-f', 'h264', tmp], check=True)
    buf = open(tmp, 'rb').read(200000)
    os.remove(tmp)
    nals = []
    for m in re.finditer(b'\x00\x00\x00\x01|\x00\x00\x01', buf):
        nals.append(m.end())
    sps = pps = None
    for k, s in enumerate(nals):
        e = nals[k + 1] - 4 if k + 1 < len(nals) else len(buf)
        t = buf[s] & 0x1f
        if t == 7 and sps is None:
            sps = buf[s:e]
        elif t == 8 and pps is None:
            pps = buf[s:e]
        if sps and pps:
            break
    if not (sps and pps):
        raise SystemExit('参考文件里没找到 SPS/PPS')
    print('  参考 SPS %d 字节 / PPS %d 字节 (profile=%d level=%d)' % (len(sps), len(pps), sps[1], sps[3]))
    return [sps], [pps]


def main():
    broken, ref, out = sys.argv[1], sys.argv[2], sys.argv[3]
    fps = sys.argv[4] if len(sys.argv) > 4 else '15'
    print('参考:', os.path.basename(ref))
    sps, pps = get_sps_pps(ref)
    hdr = b''.join(b'\x00\x00\x00\x01' + x for x in sps + pps)

    d = open(broken, 'rb').read()
    i = d.find(b'mdat')
    if i < 0:
        raise SystemExit('找不到 mdat')
    p = i + 4
    raw = out + '.h264'
    n = idr = 0
    with open(raw, 'wb') as f:
        while p + 4 <= len(d):
            L = struct.unpack('>I', d[p:p + 4])[0]
            if L == 0 or p + 4 + L > len(d):
                print('  尾部 0x%x 处截断, 丢掉最后 %d 字节(不完整的一帧)' % (p, len(d) - p))
                break
            nal = d[p + 4:p + 4 + L]
            t = nal[0] & 0x1f
            if t == 5:
                f.write(hdr); idr += 1          # 每个 IDR 前补参数集 => 可随处 seek
            f.write(b'\x00\x00\x00\x01' + nal)
            n += 1; p += 4 + L
    print('  写出 %d 个 NAL (%d 个 IDR) -> %s' % (n, idr, os.path.basename(raw)))

    cmd = ['ffmpeg', '-loglevel', 'error', '-y', '-f', 'h264', '-framerate', fps,
           '-i', raw, '-c', 'copy', '-movflags', '+faststart', out]
    print('  ' + ' '.join(cmd))
    subprocess.run(cmd, check=True)
    os.remove(raw)
    print('  ✅', out)


main()
