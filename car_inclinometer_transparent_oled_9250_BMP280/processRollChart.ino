

float chartBars[chartRollWidth];
int chartBarsPointer = 0;
float maxChartValue = 0, minChartValue = 0;
int maxChartValueIndex = 0, minChartValueIndex = 0;
bool chartBarCycle = false;
float curr_roll_angle_smooth_accum = 0.00;
int curr_roll_angle_smooth_counter = 0;

void process_roll_chart_data() {
  curr_roll_angle_smooth_accum += rollOutput;
  curr_roll_angle_smooth_counter++;
}

void calc_roll_chart_data() {
  float value = (curr_roll_angle_smooth_accum / curr_roll_angle_smooth_counter);  //(M_PI / 180) *
  bool update = false;

  //Serial.print(" value=");Serial.println(value);

  chartBars[chartBarsPointer] = value;

  if (chartBarCycle) {
    if (chartBarsPointer == maxChartValueIndex || chartBarsPointer == minChartValueIndex) {
      maxChartValue = chartBars[0];
      minChartValue = chartBars[0];

      for (int i = 0; i < chartRollWidth; i++) {
        if (chartBars[i] > maxChartValue) {
          maxChartValue = chartBars[i];
          maxChartValueIndex = chartBarsPointer;
        }

        if (chartBars[i] < minChartValue) {
          minChartValue = chartBars[i];
          minChartValueIndex = chartBarsPointer;
        }
      }

      update = true;
    }
  }

  if (value > maxChartValue) {
    maxChartValue = value;
    maxChartValueIndex = chartBarsPointer;

    update = true;
  }

  if (value < minChartValue) {
    minChartValue = value;
    minChartValueIndex = chartBarsPointer;

    update = true;
  }

  if (update && serialDebug) {
    Serial.print(" minChartValue=");
    Serial.print(minChartValue);
    Serial.print(" maxChartValue=");
    Serial.println(maxChartValue);
  }

  //Serial.print(" chartBarsPointer=");Serial.println(chartBarsPointer);
  //Serial.print(" bar=");Serial.println(chartBars[chartBarsPointer]);


  if (chartBarsPointer >= chartRollWidth - 1) {
    chartBarCycle = true;
    chartBarsPointer = 0;
  } else {
    chartBarsPointer++;
  }

  //Serial.print(" chartBarsPointerNext=");Serial.println(chartBarsPointer);

  curr_roll_angle_smooth_accum = 0;
  curr_roll_angle_smooth_counter = 0;
}

void updateBigDisplayRollChart() {
  int bottomYPosition = chartRollY + chartRollH/2;
  int maxHeight = chartRollH * 0.45;
  int xDiff = 0;

  float realMaxChartValue = max((float)6, max(abs(maxChartValue), abs(minChartValue)));
  float realMaxChartValueMapHigh = realMaxChartValue * 1000;
  float realMaxChartValueMapLow = -1 * realMaxChartValue * 1000;
  float maxHeightMapHigh = maxHeight * 1000;
  float maxHeightMapLow = -1 * maxHeight * 1000;

  int barIndex = 0;
  int barValue = 0;

  if (chartBarCycle) {
    for (int i = chartBarsPointer; i < chartRollWidth; i++) {
      barValue = int(map(chartBars[i] * 1000, realMaxChartValueMapLow, realMaxChartValueMapHigh, maxHeightMapLow, maxHeightMapHigh) / 1000 + 0.5);

      Paint_DrawLine(barIndex + xDiff, bottomYPosition, barIndex + xDiff, bottomYPosition - barValue, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

      barIndex++;
      //Serial.print(F(","));
    }
  } else {
    Paint_DrawLine(chartRollX, bottomYPosition, chartRollX + chartBarsPointer, bottomYPosition, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(chartRollX + chartBarsPointer, bottomYPosition, chartRollX + chartRollWidth, bottomYPosition, WHITE, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
  }

  for (int i = 0; i < chartBarsPointer; i++) {
    barValue = int(map(chartBars[i] * 1000, realMaxChartValueMapLow, realMaxChartValueMapHigh, maxHeightMapLow, maxHeightMapHigh) / 1000 + 0.5);

    Paint_DrawLine(barIndex + xDiff, bottomYPosition, barIndex + xDiff, bottomYPosition - barValue, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

    //Serial.print(F(","));
    barIndex++;
  }
}

void updateBigDisplayRollMinMaxData() {
  String xSign;
  String ySign;

  int aXValue = int(maxChartValue + 0.5);
  int aYValue = int(minChartValue + 0.5);

  if (aXValue > 0) {
    xSign = '+';
  } else if (aXValue < 0) {
    xSign = '-';
  } else {
    xSign = String("");
  }
  if (aYValue > 0) {
    ySign = '+';
  } else if (aYValue < 0) {
    ySign = '-';
  } else {
    ySign = String("");
  }

  String aX = xSign + String(abs(aXValue));
  String aY = ySign + String(abs(aYValue));

  int xPos;
  float hAlign;

  if (!chartBarCycle && chartBarsPointer < chartRollWidth/2) {
    xPos = chartRollX + chartRollW - 2;
    hAlign = 1;
  } else {
    xPos = chartRollX + 2;
    hAlign = 0;
  }

  Paint_DrawString(xPos, chartRollY + 2, aX, &Font8, hAlign, 0, BLACK, WHITE, 2, 2);
  Paint_DrawString(xPos, chartRollY + chartRollH - 2, aY, &Font8, hAlign, 1, BLACK, WHITE, 2, 2);
}