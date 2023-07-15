

//For RP2040 Zero select board: Raspberry Pi Pico W and USB Stack: Adafruit TinyUSB. MMC5603 library: Adafruit MMC56x3.

#include <TinyUSB_Mouse_and_Keyboard.h>
#include <OneButton.h>
#include <Adafruit_MMC56x3.h>
#include <SimpleKalmanFilter.h>

Adafruit_MMC5603 mag = Adafruit_MMC5603();
SimpleKalmanFilter xFilter(1, 1, 0.2), yFilter(1, 1, 0.2), zFilter(1, 1, 0.2);

// 设置按键
OneButton button1(27, true);
OneButton button2(26, true);

float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;

int calSamples = 300;
int sensivity = 20000; // 光标移动灵敏度
int magRange = 3;
int outRange = 127; // HID报告中允许的最大值
float xyThreshold = 400; // 中心阈值

int inRange = magRange * sensivity;
float zThreshold = 400;

bool isOrbit = false;

void setup()
{

button1.attachClick(goHome);
button1.attachLongPressStop(goHome);

button2.attachClick(fitToScreen);
button2.attachLongPressStop(fitToScreen);

// 鼠标和键盘初始化
Mouse.begin();
Keyboard.begin();

Serial.begin(9600);
Wire1.begin();

// 磁传感器初始化
mag.begin(MMC56X3_DEFAULT_ADDRESS, &Wire);

// 首次启动时的原油偏移量校准
for (int i = 1; i <= calSamples; i++)
{
mag.setDataRate(100);
mag.setContinuousMode(true);

sensors_event_t event;
delay(mag.getEvent(&event));
xOffset += event.magnetic.x;
yOffset += event.magnetic.y;
zOffset += event.magnetic.z;

Serial.print(".");
}

xOffset = xOffset / calSamples;
yOffset = yOffset / calSamples;
zOffset = zOffset / calSamples;

Serial.println();
Serial.println(xOffset);
Serial.println(yOffset);
Serial.println(zOffset);
}

void loop()
{

// keep watching the push buttons
button1.tick();
button2.tick();

// get the mag data
sensors_event_t event;
delay(mag.getEvent(&event));
mag.setContinuousMode(true);

// 更新筛选器
xCurrent = xFilter.updateEstimate(event.magnetic.x - xOffset);
yCurrent = yFilter.updateEstimate(event.magnetic.y - yOffset);
zCurrent = zFilter.updateEstimate(event.magnetic.z - zOffset);

// 检查中心阈值
if (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold)
{

int xMove = 0;
int yMove = 0;

// 将磁力计xy映射到HID repports中允许的127范围
xMove = map(xCurrent, -inRange, inRange, -outRange, outRange);
yMove = map(yCurrent, -inRange, inRange, -outRange, outRange);

// 如果未越过平移阈值，则在Fusion 360中按下“平移到轨道” (zAxis)
if (abs(zCurrent) < zThreshold && !isOrbit)
{
Keyboard.press(KEY_LEFT_SHIFT);
isOrbit = false;
}

// 通过按住鼠标中键并与xy轴成比例移动来平移或动态观察
Mouse.press(MOUSE_MIDDLE);
Mouse.move(-yMove, -xMove, 0);
}
else
{

// 如果在中心阈值内，释放鼠标和键盘
Mouse.release(MOUSE_MIDDLE);
Keyboard.releaseAll();
isOrbit = true;
}
Serial.print("X: ");
Serial.print(xCurrent);
Serial.print(", ");
Serial.print("Y: ");
Serial.print(yCurrent);
Serial.print(", ");
Serial.print("Z: ");
Serial.print(zCurrent);
Serial.println();
}

// 按指定给自定义外接程序命令的（CMD+SHIFT+H）快捷键，转到Fusion 360中的主视
void goHome()
{
Keyboard.press(KEY_LEFT_SHIFT);
Keyboard.write('h');

delay(10);
Keyboard.releaseAll();
Serial.println("pressed home");
}

// 按两次鼠标中键以适应视图
void fitToScreen()
{
Mouse.press(MOUSE_MIDDLE);
Mouse.release(MOUSE_MIDDLE);
Mouse.press(MOUSE_MIDDLE);
Mouse.release(MOUSE_MIDDLE);

Serial.println("pressed fit");
}
