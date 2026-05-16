# UI State Diagram Style (Graphviz)

Use this style to render UI states with a rounded outer state box and a square LCD text box inside it, using a fixed-width font. All text is centered (no padding).

Canonical diagram:
- `docs/ui_state_machine_lcd.dot` (source)
- `docs/ui_state_machine_lcd.png` (rendered)

## Graph settings
```
digraph UiStateLCD {
  rankdir=LR;
  node [shape=box, style="rounded"];

  // ... nodes + edges ...
}
```

## Node template
```
UI_EXAMPLE [label=<<FONT FACE="Courier"><TABLE BORDER="0" CELLBORDER="0" CELLSPACING="0">
  <TR><TD ALIGN="CENTER"><B>UI_EXAMPLE</B></TD></TR>
  <TR><TD>
    <TABLE BORDER="1" CELLBORDER="0" CELLSPACING="0" CELLPADDING="2">
      <TR><TD ALIGN="CENTER">Line 1</TD></TR>
      <TR><TD ALIGN="CENTER">Line 2</TD></TR>
      <TR><TD ALIGN="CENTER">Line 3</TD></TR>
      <TR><TD ALIGN="CENTER">Line 4</TD></TR>
    </TABLE>
  </TD></TR>
</TABLE></FONT>>];
```

## Notes
- Keep LCD text lines trimmed; don’t pad with spaces.
- Use `&#160;` if you need a blank line that still has height.
- Escape `&` as `&amp;` in labels.

## Render command
```
dot -Tpng docs/ui_state_machine_lcd.dot -o docs/ui_state_machine_lcd.png
```
