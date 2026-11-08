# 演示素材

| 文件 | 状态 | 内容 | 规格 |
|---|---|---|---|
| `demo.gif` | 已完成 | 实拍视频前 20 秒循环动图，README 首屏自动播放 | 400×400，100 帧 @ 5 fps，4.2 MB，无限循环 |
| `demo.mp4` | 已完成 | 完整运行视频（实拍原片转码） | 37 秒，720×1280 竖屏，H.264，5.2 MB，无音轨 |
| `hero.jpg` | 待补 | 整车正面/斜 45° 清晰照片，能看到 PCB、电机和 MPU6050 | 1200 × 800，光线充足、背景干净 |
| `vofa-waveform.png` | 待补 | VOFA+ 七通道实时波形截图 | 保留坐标轴与通道名，能看清 8.9 ms 周期通道 |
| `wiring.jpg` | 可选 | 接线特写 | 标注关键引脚 |
| `social-preview.png` | 可选 | GitHub 仓库分享卡片图 | 1280 × 640，在仓库 Settings → Social preview 上传 |

## 素材来源与制作记录

原始素材是一段竖屏实拍视频：HEVC（hvc1）、720×1280、37.43 秒、1.7 MB。做了两处处理：

1. **完整视频转 H.264**。HEVC 在浏览器里兼容性不稳定，GitHub 的视频预览需要 H.264；`-movflags +faststart` 把 moov 挪到文件头，便于边下边播。去掉音轨以减小体积。
2. **首屏 GIF**。竖屏上下留白较多，裁成方形（取垂直 y = 180~900，覆盖小车全程位置）后缩放到 400×400。用两遍调色板优化（64 色、`stats_mode=diff`、无抖动）把首版 13.6 MB 压到 4.2 MB。

复现命令（ffmpeg 7.1，输入 `original.mp4` 为原始竖屏片）：

```bash
# 完整视频 → H.264
ffmpeg -i original.mp4 -c:v libx264 -b:v 1100k -pix_fmt yuv420p -an \
  -movflags +faststart -y demo.mp4

# 首屏 GIF：调色板 + 使用（两遍）
ffmpeg -t 20 -i original.mp4 \
  -vf "crop=720:720:0:180,fps=5,scale=400:-1:flags=lanczos,palettegen=max_colors=64:stats_mode=diff" \
  -y palette.png
ffmpeg -t 20 -i original.mp4 -i palette.png \
  -lavfi "crop=720:720:0:180,fps=5,scale=400:-1:flags=lanczos[v];[v][1:v]paletteuse=dither=none" \
  -loop 0 -y demo.gif
```

> 本机 ffmpeg 未带 `libx264`，实际用的是 Windows Media Foundation 的 `h264_mf` 编码器；在有 `libx264` 的环境里用上面的命令即可。原始 HEVC 片保留在本地工作区的 `build/` 下，未纳入仓库。

## README 引用方式

`README.md` 的「实物演示」章节引用 `demo.gif`（自动播放）并给出 `demo.mp4` 的链接（点击后 GitHub 会打开视频播放器）。两个文件都是相对路径，随仓库一起上传即可，不依赖外部图床或 CDN。

后续补拍时建议：固定机位、侧向略带俯视、地面留参照物（能看出「走回原位」）、避免强曝光；手机遥控按键如果能入镜，可信度更高。
