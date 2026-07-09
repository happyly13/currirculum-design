#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/* SSD1306 I2C address (7-bit: 0x3C, shifted: 0x78) */
#define SSD1306_ADDR        0x3C

/* Display dimensions */
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64
#define SSD1306_PAGES       8

/* Color constants */
#define OLED_BLACK          0
#define OLED_WHITE          1

/* Framebuffer - accessible for direct manipulation */
extern uint8_t OLED_Buffer[SSD1306_WIDTH * SSD1306_PAGES];

/* Icon / bitmap exports for UI layer */
extern const uint8_t IconHeart[20];
extern const uint8_t IconBattery[12];
extern const uint8_t IconBT[8];

/* Font dimensions */
#define FONT_6X8_WIDTH      6
#define FONT_6X8_HEIGHT     8
#define FONT_8X16_WIDTH     8
#define FONT_8X16_HEIGHT    16
#define FONT_16X24_WIDTH    16
#define FONT_16X24_HEIGHT   24

/* ==================== OLED Driver API ==================== */

/* ---- 初始化与控制 ---- */

/** @brief 初始化OLED显示屏
 *  执行上电延时(100ms) → SSD1306寄存器配置序列 → 清屏
 *  配置包括：水平寻址模式、段重映射、COM扫描方向、电荷泵使能、对比度等
 */
void OLED_Init(void);

/** @brief 清空帧缓冲区并立即刷新到屏幕
 *  等效于调用 OLED_ClearBuffer() + OLED_Update()
 */
void OLED_Clear(void);

/** @brief 仅清空帧缓冲区（全部置0），不刷新到屏幕
 *  用于准备新的一帧画面，通常搭配 OLED_Update() 使用
 */
void OLED_ClearBuffer(void);

/** @brief 将整个帧缓冲区(1024字节)逐页发送到OLED显示屏
 *  共8页(Page 0~7)，每页128列，使用页寻址模式传输
 */
void OLED_Update(void);

/** @brief 局部刷新指定页范围到OLED显示屏
 *  @param start_page 起始页号(0~7)
 *  @param end_page   结束页号(0~7)，仅刷新 [start_page, end_page] 范围的页
 */
void OLED_UpdateArea(uint8_t start_page, uint8_t end_page);

/** @brief 设置OLED对比度
 *  @param contrast 对比度值(0~255)，值越大显示越亮
 */
void OLED_SetContrast(uint8_t contrast);

/** @brief 开启OLED显示（发送 0xAF 命令） */
void OLED_DisplayOn(void);

/** @brief 关闭OLED显示（发送 0xAE 命令），进入睡眠模式 */
void OLED_DisplayOff(void);

/** @brief 设置显示反色
 *  @param invert 0=正常显示(亮底暗字), 1=反色显示(暗底亮字)
 */
void OLED_InvertDisplay(uint8_t invert);

/* ---- 绘图基元（操作帧缓冲区，需调用 OLED_Update 才能显示）---- */

/** @brief 在帧缓冲区中绘制单个像素
 *  @param x     水平坐标(0~127)
 *  @param y     垂直坐标(0~63)
 *  @param color OLED_BLACK(0)=擦除, OLED_WHITE(1)=点亮
 */
void OLED_SetPixel(uint8_t x, uint8_t y, uint8_t color);

/** @brief 使用Bresenham算法绘制任意角度直线
 *  @param x0,y0 起点坐标
 *  @param x1,y1 终点坐标
 */
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

/** @brief 绘制水平直线
 *  @param x 起点x坐标
 *  @param y y坐标
 *  @param w 线段宽度(像素)
 */
void OLED_DrawHLine(uint8_t x, uint8_t y, uint8_t w);

/** @brief 绘制垂直直线
 *  @param x x坐标
 *  @param y 起点y坐标
 *  @param h 线段高度(像素)
 */
void OLED_DrawVLine(uint8_t x, uint8_t y, uint8_t h);

/** @brief 绘制空心矩形边框
 *  @param x,y 左上角坐标
 *  @param w   矩形宽度
 *  @param h   矩形高度
 */
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/** @brief 绘制实心填充矩形
 *  @param x,y 左上角坐标
 *  @param w   矩形宽度
 *  @param h   矩形高度
 */
void OLED_DrawFilledRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);

/** @brief 使用中点圆算法绘制空心圆
 *  @param cx,cy 圆心坐标
 *  @param r     半径(像素)
 */
void OLED_DrawCircle(uint8_t cx, uint8_t cy, uint8_t r);

/** @brief 绘制实心填充圆
 *  @param cx,cy 圆心坐标
 *  @param r     半径(像素)
 */
void OLED_DrawFilledCircle(uint8_t cx, uint8_t cy, uint8_t r);

/** @brief 在帧缓冲区中绘制位图
 *  @param x,y     位图左上角坐标
 *  @param bitmap  位图数据指针（按页排列，每字节8个纵向像素）
 *  @param w       位图宽度
 *  @param h       位图高度
 */
void OLED_DrawBitmap(uint8_t x, uint8_t y, const uint8_t *bitmap, uint8_t w, uint8_t h);

/* ---- 文字渲染：6x8 字体（页对齐，ASCII 0x20~0x7E）---- */

/** @brief 在指定页绘制单个6x8字符
 *  @param x    水平像素坐标(0~122)，超出部分自动裁剪
 *  @param page 页号(0~7)，每页8像素高
 *  @param c    要显示的ASCII字符(0x20~0x7E)，超出范围显示空格
 */
void OLED_DrawChar6x8(uint8_t x, uint8_t page, char c);

/** @brief 在指定页绘制6x8字符串
 *  @param x    起始水平像素坐标
 *  @param page 页号(0~7)
 *  @param str  以'\0'结尾的字符串，超出屏幕宽度的部分自动截断
 */
void OLED_DrawString6x8(uint8_t x, uint8_t page, const char *str);

/* ---- 文字渲染：8x16 字体（页对齐，支持0-9、A-Z、: - . / 空格）---- */

/** @brief 在指定页绘制单个8x16字符（占用2页）
 *  @param x    水平像素坐标(0~120)
 *  @param page 起始页号(0~6)，字符占用 page 和 page+1 两页
 *  @param c    要显示的字符：0-9、A-Z/a-z（大小写均显示大写）、: . - / 空格
 */
void OLED_DrawChar8x16(uint8_t x, uint8_t page, char c);

/** @brief 在指定页绘制8x16字符串
 *  @param x    起始水平像素坐标
 *  @param page 起始页号(0~6)
 *  @param str  以'\0'结尾的字符串
 */
void OLED_DrawString8x16(uint8_t x, uint8_t page, const char *str);

/* ---- 大号数字渲染：16x24 字体（支持非页对齐，用于时钟显示）---- */

/** @brief 在指定像素坐标绘制16x24大号数字（0~9）
 *  支持非页对齐的y坐标，自动处理跨页位偏移
 *  @param x     水平像素坐标(0~111)
 *  @param y     垂直像素坐标(0~40)
 *  @param digit 数字(0~9)，超出范围不绘制
 */
void OLED_DrawDigit16x24(uint8_t x, uint8_t y, uint8_t digit);

/** @brief 在指定像素坐标绘制16x24大号冒号(:)
 *  @param x 水平像素坐标
 *  @param y 垂直像素坐标
 */
void OLED_DrawColon16x24(uint8_t x, uint8_t y);

/** @brief 快速绘制 HH:MM 格式的时钟界面
 *  自动布局：x → HH十位 → HH个位 → 冒号 → MM十位 → MM个位
 *  @param x      起始水平像素坐标
 *  @param y      垂直像素坐标
 *  @param hour   小时(0~23)
 *  @param minute 分钟(0~59)
 */
void OLED_DrawTime16x24(uint8_t x, uint8_t y, uint8_t hour, uint8_t minute);

/* ---- 便捷输出 ---- */

/** @brief 格式化字符串输出（类printf），使用6x8字体
 *  @param x    起始水平像素坐标
 *  @param page 页号(0~7)
 *  @param fmt  printf格式字符串（内部缓冲区32字节，超出会截断）
 *  @param ...  可变参数
 */
void OLED_Printf6x8(uint8_t x, uint8_t page, const char *fmt, ...);

#endif /* __OLED_H */
