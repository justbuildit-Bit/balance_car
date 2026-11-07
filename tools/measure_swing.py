# -*- coding: utf-8 -*-
"""
平衡车摆动频率测量（视频像素法）v2

用法：
    python measure_swing.py 视频.mp4
    python measure_swing.py 视频.mp4 --lo 0.3 --hi 4.0 --plot 图.png --csv 数据.csv

v2 相对 v1 的三处改动（都是被一次坏数据逼出来的）：
  1. 新增谱峰锐度 Q。v1 只看 SNR，结果遇到"车在慢慢漂移"时 SNR 高达 42，
     误判成 0.56 Hz 振荡。Q = f_peak / 半高带宽，窄带振荡 Q>2.5，
     随机漂移 Q<2。
  2. 新增机位稳定性检测（相位相关累加位移）。手持跟拍会让"相机运动"
     混进"车的运动"，频率直接作废。
  3. 结论改成三档：可用 / 存疑 / 不可用，并打印原因。

测量做法（判据能成立的前提）：
  · 手机固定（靠墙/支架/书堆），全程别碰
  · 拔掉车上排线，避免线束给车体额外的侧向约束
  · 车放画面中央，留足漂移空间，拍 15~20 s
  · 中途不碰车；车倒了或漂出画面就重拍

局限：本脚本只能给出频率，给不出角度。像素→角度的标定依赖参照物，
实测在 1.4~2.7 px/mm 之间浮动，换算出的角度误差可达一倍。
要精确幅值请用 VOFA+ 的传感器数据（50 Hz、无机位问题）。
"""
import argparse
import numpy as np
import cv2


def track_battery(path, min_area=200):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit('打不开视频：%s' % path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cx, cy, area = [], [], []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        b, g, r = cv2.split(fr.astype(int))
        mask = ((b > 110) & (b - r > 25) & (b - g > 10)).astype(np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        ys, xs = np.nonzero(mask)
        if len(xs) < min_area:
            cx.append(np.nan)
            cy.append(np.nan)
            area.append(0)
        else:
            cx.append(xs.mean())
            cy.append(ys.mean())
            area.append(len(xs))
    cap.release()
    return (np.array(cx, float), np.array(cy, float),
            np.array(area, int), fps, (w, h))


def camera_motion(path):
    """用相位相关累加全局平移，判断机位是否固定。"""
    cap = cv2.VideoCapture(path)
    prev = None
    d = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY).astype(np.float32)
        g = cv2.resize(g, (360, 640))
        if prev is not None:
            (dx, dy), _ = cv2.phaseCorrelate(prev, g)
            d.append((dx * 2.0, dy * 2.0))     # 还原到原始尺度
        prev = g
    cap.release()
    if not d:
        return 0.0, 0.0
    d = np.array(d)
    mag = np.hypot(d[:, 0], d[:, 1])
    secs = len(d) / 30.0
    return float(np.median(mag)), float(mag.sum() / max(secs, 1e-6))


def analyze(s, fps, lo, hi):
    n = len(s)
    t = np.arange(n) / fps
    if np.isnan(s).any():
        ok = ~np.isnan(s)
        s = np.interp(np.arange(n), np.arange(n)[ok], s[ok])
    s = s - np.polyval(np.polyfit(t, s, 1), t)      # 去线性漂移
    s = s - s.mean()

    F = np.fft.rfft(s * np.hanning(n))
    f = np.fft.rfftfreq(n, 1.0 / fps)
    mag = np.abs(F)

    band = (f >= lo) & (f <= hi)
    if not band.any():
        raise SystemExit('频段 [%g, %g] Hz 内无频点，视频太短' % (lo, hi))

    base = np.median(mag[band])

    def peak_metrics(k):
        """给一个谱峰算 SNR 和 Q（半高全宽）。"""
        far = band & (np.abs(f - f[k]) > 0.15)
        noise = np.median(mag[far]) if far.any() else 1e-9
        snr = mag[k] / (noise + 1e-12)
        height = mag[k] - base
        half = base + height / 2.0
        i = k
        while i > 0 and mag[i] > half:
            i -= 1
        j = k
        while j < len(mag) - 1 and mag[j] > half:
            j += 1
        bw = max(f[j] - f[i], 1e-6)
        return snr, f[k] / bw, bw

    # 带内所有局部极大
    idx = np.where(band)[0]
    peaks = [k for k in idx
             if 0 < k < len(mag) - 1 and mag[k] >= mag[k - 1] and mag[k] > mag[k + 1]]
    if not peaks:
        peaks = [int(np.argmax(np.where(band, mag, 0.0)))]

    cand = []
    for k in peaks:
        snr, Q, bw = peak_metrics(k)
        cand.append(dict(k=k, f=f[k], snr=snr, Q=Q, bw=bw, mag=mag[k]))

    # 优先选"窄峰"（真振荡），窄峰都不显著时退回最显著的峰并标宽包
    narrow = [c for c in cand if c['Q'] >= 2.5 and c['snr'] >= 3.0]
    if narrow:
        pick = max(narrow, key=lambda c: c['snr'])
        is_narrow = True
    else:
        pick = max(cand, key=lambda c: c['snr'])
        is_narrow = False

    fpk = pick['f']
    m2 = np.abs(f - fpk) <= 0.35
    Fb = np.zeros_like(F)
    Fb[m2] = F[m2]
    y = np.fft.irfft(Fb, n)

    return dict(f=fpk, period=1.0 / fpk, snr=pick['snr'], Q=pick['Q'],
                bw=pick['bw'], is_narrow=is_narrow, n_cand=len(cand),
                cand=sorted(cand, key=lambda c: -c['snr'])[:4],
                pp=y.max() - y.min(), rms=y.std(),
                n=n, df=fps / n, cyc=(n / fps) * fpk)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('video')
    ap.add_argument('--lo', type=float, default=0.6)
    ap.add_argument('--hi', type=float, default=4.0)
    ap.add_argument('--plot', default=None)
    ap.add_argument('--csv', default=None)
    a = ap.parse_args()

    cx, cy, area, fps, (w, h) = track_battery(a.video)
    n = len(cx)
    valid = int((~np.isnan(cx)).sum())
    dur = n / fps
    med_shift, speed = camera_motion(a.video)

    print('=' * 66)
    print('视频：%s' % a.video)
    print('分辨率 %dx%d   帧率 %.2f fps   帧数 %d   时长 %.2f s' % (w, h, fps, n, dur))
    print('电池检出 %d/%d 帧（%.0f%%）  频率分辨率 %.3f Hz'
          % (valid, n, 100.0 * valid / max(n, 1), fps / n))
    print('机位稳定度：相邻帧平移中位 %.2f px，累计 %.0f px/s' % (med_shift, speed))
    print('-' * 66)
    print('%-6s %8s %8s %8s %8s %6s %6s' % ('信号', '主频Hz', '周期s', '峰峰px', 'SNR', 'Q', '周期数'))

    best = None
    for name, s in [('电池X', cx), ('电池Y', cy)]:
        r = analyze(s, fps, a.lo, a.hi)
        print('%-6s %8.2f %8.3f %8.2f %8.1f %6.1f %6.1f'
              % (name, r['f'], r['period'], r['pp'], r['snr'], r['Q'], r['cyc']))
        if best is None or r['snr'] > best[1]['snr']:
            best = (name, r)

    name, r = best
    print('-' * 66)
    print('带内候选峰（按显著度）：' +
          '  '.join('%.2fHz(SNR%.0f,Q%.1f)' % (c['f'], c['snr'], c['Q']) for c in r['cand']))
    print('-' * 66)

    problems = []
    if valid < 0.9 * n:
        problems.append('电池有 %d 帧未检出（遮挡或移出画面）' % (n - valid))
    if speed > 60:
        problems.append('机位在移动（%.0f px/s，固定机位应在 40 以下）→ 相机运动混进了车的运动' % speed)
    if not r['is_narrow']:
        problems.append('带内没有窄峰（最优峰 Q=%.1f，窄带振荡应 ≥2.5）→ 这是漂移或随机游走，不是周期振荡' % r['Q'])
    if r['snr'] < 3:
        problems.append('信号不显著（SNR=%.1f < 3）' % r['snr'])
    if dur < 5:
        problems.append('时长只有 %.1f s，不到 3 个周期，无法定频' % dur)

    if problems:
        print('判定：❌ 数据不可用')
        for p in problems:
            print('   · ' + p)
        print()
        print('  重拍要求：手机固定不动 / 拔掉车上排线 / 车放画面中央留出漂移空间 /')
        print('            拍 15~20 s / 中途不碰车、不出画面、不倒下')
    else:
        print('判定：✅ 数据可用')
        if r['Q'] >= 2.5 and r['snr'] >= 5:
            print('  信号质量好（Q=%.1f，SNR=%.1f），主频可采信' % (r['Q'], r['snr']))
        print('  以 %s 为准：主频 %.2f Hz（周期 %.3f s，%d 个完整周期）'
              % (name, r['f'], r['period'], int(r['cyc'])))
        if dur < 10:
            print('  注：时长 %.1f s，频率分辨率 %.2f Hz，建议下次拍 ≥10 s' % (dur, fps / n))
        print('  对照基线：拔线地面 1.56 Hz；有线 1.69~1.81 Hz')
        print('  判读方法：改一个参数重测，看频率是否跟着移动。')
        print('            移动 → 振荡由该环节决定；不移动 → 改的只是系统软硬，不是根源。')
    print('=' * 66)

    if a.csv:
        t = np.arange(n) / fps
        np.savetxt(a.csv, np.column_stack([t, cx, cy]),
                   delimiter=',', header='t_s,cx_px,cy_px', comments='')
        print('已保存数据：%s' % a.csv)

    if a.plot:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from matplotlib import font_manager
            for fam in ['Microsoft YaHei', 'SimHei', 'DengXian']:
                try:
                    font_manager.findfont(fam, fallback_to_default=False)
                    plt.rcParams['font.sans-serif'] = [fam]
                    break
                except Exception:
                    pass
            plt.rcParams['axes.unicode_minus'] = False
            t = np.arange(n) / fps
            fig, ax = plt.subplots(2, 1, figsize=(9, 5.0), sharex=True)
            tag = '不可用' if problems else '可用'
            ax[0].plot(t, cx, color='#17457A', lw=1.0)
            ax[0].set_ylabel('电池 X (px)', fontsize=9)
            ax[0].set_title('[%s] 主频 %.2f Hz  周期 %.3f s  Q %.1f  SNR %.1f'
                            % (tag, r['f'], r['period'], r['Q'], r['snr']),
                            fontsize=11, loc='left',
                            color='#A8442A' if problems else '#16202B')
            ax[1].plot(t, cy, color='#A8442A' if not problems else '#9BB3CC', lw=1.0)
            ax[1].set_ylabel('电池 Y (px)', fontsize=9)
            ax[1].set_xlabel('时间 (s)', fontsize=9)
            for x in ax:
                x.grid(alpha=.18, lw=.6)
                x.tick_params(labelsize=8)
                for sp in ('top', 'right'):
                    x.spines[sp].set_visible(False)
            plt.tight_layout()
            plt.savefig(a.plot, dpi=150, facecolor='white')
            print('已保存图片：%s' % a.plot)
        except Exception as e:
            print('画图失败（不影响频率结果）：%s' % e)


if __name__ == '__main__':
    main()
