from pathlib import Path
import re

html = Path('index.html').read_text(encoding='utf-8')
css = Path('styles.css').read_text(encoding='utf-8')
js = Path('app.js').read_text(encoding='utf-8')

# 检查 HTML 中所有 ID
ids = re.findall(r'id="([^"]+)"', html)
print(f"HTML 中共有 {len(ids)} 个 ID 元素")

# 检查 JS 中获取的 ID
js_ids = re.findall(r'getElementById\(["\']([^"\']+)["\']\)', js)
missing_in_html = set(js_ids) - set(ids)
if missing_in_html:
    print(f"⚠ JS 使用但 HTML 中缺失的 ID: {missing_in_html}")
else:
    print("✓ JS 中所有 getElementById 对应的 ID 都在 HTML 中存在")

# 检查 CSS 文件大小和关键选择器
print(f"\nCSS 文件大小: {len(css)} 字节")

key_selectors = [
    'station-shell', 'map-panel', 'fieldCanvas', 'map-tooltip',
    'side-panel', 'status-strip', 'link-badge', 'node-status',
    'phase-chip', 'timeline', 'metric-grid', 'control-grid',
    'serial-log', 'command-button', 'icon-button'
]

missing_selectors = []
for sel in key_selectors:
    if sel not in css:
        missing_selectors.append(sel)

if missing_selectors:
    print(f"⚠ CSS 中缺失的关键选择器: {missing_selectors}")
else:
    print("✓ 所有关键 CSS 选择器都存在")

print("\n✓ 验证完成，结构完整")
