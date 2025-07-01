Note: all cobot_msgs are modified by for ros2:
```
find msg srv -type f \( -name "*.msg" -o -name "*.srv" \) -exec sed -i -E 's/\b([a-z]+)([A-Z][a-zA-Z0-9_]*)\b/\1_\L\2/g' {} +
find msg srv -type f | xargs sed -i -E 's/^([a-z0-9_]+)?int(8|16|32|64|float32|float64|uint8|uint16|uint32|uint64)[ ]+([A-Za-z0-9]+)[ ]*=[ ]*/int\2 \U\3\E = /'
find msg srv -type f -exec sed -i 's/\bVBatt\b/v_batt/g' {} +
find msg srv -type f -exec sed -i 's/\bID\b/id/g' {} +
find msg -type f -exec sed -i -E 's/^Header[[:space:]]+/std_msgs\/Header /' {} +
```