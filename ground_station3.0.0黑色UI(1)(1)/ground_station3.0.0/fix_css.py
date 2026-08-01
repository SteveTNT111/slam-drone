# -*- coding: utf-8 -*-
with open('styles.css', 'r', encoding='utf-8') as f:
    content = f.read()

old_block = """.map-panel {
  display: flex;
  flex-direction: column;
  overflow: hidden;
  transition: box-shadow 0.3s ease;
  max-width: 600px;
  justify-self: center;
  width: 100%;
}"""

new_block = """.map-panel {
  display: flex;
  flex-direction: column;
  overflow: hidden;
  transition: box-shadow 0.3s ease;
  max-width: 600px;
  width: 100%;
}"""

if old_block in content:
    content = content.replace(old_block, new_block, 1)
    with open('styles.css', 'w', encoding='utf-8') as f:
        f.write(content)
    print('Done: removed justify-self: center; from .map-panel')
else:
    print('ERROR: old block not found')
