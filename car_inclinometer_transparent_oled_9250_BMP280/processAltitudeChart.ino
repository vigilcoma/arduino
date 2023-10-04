float chartAltBars[chartRollWidth];
int chartAltBarsPointer = 0;
float maxAltChartValue = 0, minAltChartValue = 0;
int maxAltChartValueIndex = 0, minAltChartValueIndex = 0;
bool chartAltBarCycle = false;
float curr_alt_smooth_accum = 0.00;
int curr_alt_smooth_counter = 0;
bool firstAlt = true;

void process_altitude_chart_data() {
  curr_alt_smooth_accum += altitudeOutput;
  curr_alt_smooth_counter++;
}

void calc_altitude_chart_data() {
  float value = (curr_alt_smooth_accum / curr_alt_smooth_counter);
  bool update = false;

  if(firstAlt) {
    firstAlt = false;
    maxAltChartValue = value;
    minAltChartValue = value - 1;
  }

  chartAltBars[chartAltBarsPointer] = value;

  if (chartAltBarCycle) {
    if (chartAltBarsPointer == maxAltChartValueIndex || chartAltBarsPointer == minAltChartValueIndex) {
      maxAltChartValue = chartAltBars[0];
      minAltChartValue = chartAltBars[0];

      for (int i = 0; i < chartRollWidth; i++) {
        if (chartAltBars[i] > maxAltChartValue) {
          maxAltChartValue = chartAltBars[i];
          maxAltChartValueIndex = chartAltBarsPointer;
        }

        if (chartAltBars[i] < minAltChartValue) {
          minAltChartValue = chartAltBars[i];
          minAltChartValueIndex = chartAltBarsPointer;
        }
      }

      update = true;
    }
  }

  if (value > maxAltChartValue) {
    maxAltChartValue = value;
    maxAltChartValueIndex = chartAltBarsPointer;

    update = true;
  }

  if (value < minAltChartValue) {
    minAltChartValue = value;
    minAltChartValueIndex = chartAltBarsPointer;

    update = true;
  }

  if (update && serialDebug) {
    Serial.print(" minAltChartValue=");
    Serial.print(minAltChartValue);
    Serial.print(" val=");
    Serial.print(value);
    Serial.print(" maxAltChartValue=");
    Serial.println(maxAltChartValue);
  }

  //Serial.print(" chartBarsPointer=");Serial.println(chartBarsPointer);
  //Serial.print(" bar=");Serial.println(chartBars[chartBarsPointer]);


  if (chartAltBarsPointer >= chartRollWidth - 1) {
    chartAltBarCycle = true;
    chartAltBarsPointer = 0;
  } else {
    chartAltBarsPointer++;
  }

  //Serial.print(" chartBarsPointerNext=");Serial.println(chartBarsPointer);

  curr_alt_smooth_accum = 0;
  curr_alt_smooth_counter = 0;
}

void updateBigDisplayAltitudeChart() {
  int maxHeight = chartRollH * 0.9;
  int bottomYPosition = chartRollY + chartRollH - (chartRollH - maxHeight)/2;
  
  int xDiff = 0;

  int barIndex = 0;
  int barValue = 0;
  float lastX, lastY;

  float localMin, localMax, fixer;
  if(minAltChartValue < 0) {
    localMin = 0;
    localMax = maxAltChartValue + abs(minAltChartValue);
    fixer = abs(minAltChartValue);
  } else {
    localMin = minAltChartValue;
    localMax = maxAltChartValue;
    fixer = 0;
  }

  if (chartAltBarCycle) {
    for (int i = chartAltBarsPointer; i < chartRollWidth; i++) {
      barValue = int(map(chartAltBars[i] + fixer, localMin, localMax, 0, maxHeight) + 0.5);

      if(barIndex == 0) {
        lastX = barIndex + xDiff;
        lastY = bottomYPosition - barValue;

        Paint_DrawPoint(lastX, lastY, WHITE, DOT_PIXEL_1X1, DOT_FILL_AROUND);
      } else {
        Paint_DrawLine(lastX, lastY, barIndex + xDiff, bottomYPosition - barValue, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

        lastX = barIndex + xDiff;
        lastY = bottomYPosition - barValue;
      }

      barIndex++;
    }
  }


  for (int i = 0; i < chartAltBarsPointer; i++) {
    barValue = int(map(chartAltBars[i] + fixer, localMin, localMax, 0, maxHeight) + 0.5);

    if(barIndex == 0) {
      lastX = barIndex + xDiff;
      lastY = bottomYPosition - barValue;

      Paint_DrawPoint(lastX, lastY, WHITE, DOT_PIXEL_1X1, DOT_FILL_AROUND);
    } else {
      Paint_DrawLine(lastX, lastY, barIndex + xDiff, bottomYPosition - barValue, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

      lastX = barIndex + xDiff;
      lastY = bottomYPosition - barValue;
    }
    
    barIndex++;
  }
}

void updateBigDisplayAltMinMaxData() {
  String xSign;
  String ySign;

  int aXValue = int(maxAltChartValue + 0.5);
  int aYValue = int(minAltChartValue + 0.5);

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

  if (!chartAltBarCycle && chartAltBarsPointer < chartRollWidth*2/3) {
    xPos = chartRollX + chartRollW - 2;
    hAlign = 1;
  } else {
    xPos = chartRollX + 2;
    hAlign = 0;
  }

  Paint_DrawString(xPos, chartRollY + 2, aX, &Font8, hAlign, 0, BLACK, WHITE, 2, 2);
  Paint_DrawString(xPos, chartRollY + chartRollH - 2, aY, &Font8, hAlign, 1, BLACK, WHITE, 2, 2);
}