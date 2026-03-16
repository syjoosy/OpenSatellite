
void draw_digit_5x3(int digit, int x, int y) 
{
    //if (digit < 0 || digit > 9) return; // проверка

	if (digit == -1)
	{
		for (int row = 0; row < 5; row++) {
			for (int col = 0; col < 3; col++) {
				int px1 = x + col * STEP;
				int py1 = y + row * STEP;
				int px2 = px1 + CELL_SIZE - 1;
				int py2 = py1 + CELL_SIZE - 1;

				if (colon_5x3[row][col]) {
					epd_paint_drawRectangle(px1, py1, px2, py2, EPD_COLOR_BLACK, 1);
				}
			}
		}
		return;
	}
}

void draw_digit_7x5(int digit, int x, int y)
{
  if (digit < 0 || digit > 9)
    return;

  for (int row = 0; row < 7; row++)
  {
    for (int col = 0; col < 5; col++)
    {
      int px1 = x + col * STEP;
      int py1 = y + row * STEP;
      int px2 = px1 + CELL_SIZE - 1;
      int py2 = py1 + CELL_SIZE - 1;

      if (digit_7x5[digit][row][col])
      {
        epd_paint_drawRectangle(px1, py1, px2, py2, EPD_COLOR_BLACK, 1);
      }
    }
  }
}