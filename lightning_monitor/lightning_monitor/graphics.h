/***************************************************************************//**
* @file    graphics.h
* @version 1.0.0
* @authors Jaroslav Groman
*
* @par Project Name
*
*
* @par Description
*    .
*
* @par Target device
*    Azure Sphere MT3620
*
* @par Related hardware
*    Avnet Azure Sphere Starter Kit
*
* @par Code Tested With
*    1. Silicon: Avnet Azure Sphere Starter Kit
*    2. IDE: Visual Studio 2017
*    3. SDK: Azure Sphere SDK Preview
*
* @par Notes
*    .
*
*******************************************************************************/

#ifndef GRAPHICS_H
#define GRAPHICS_H

#ifdef __cplusplus
extern "C" {
#endif

#define XBM_LOGO_W 65
#define XBM_LOGO_H 64
static const unsigned char XBM_LOGO_BITS[] = {
   0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0,
   0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x1e, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x0c, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
   0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0xc0, 0xc0,
   0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x80, 0xf9, 0x0f, 0x00, 0x00,
   0x00, 0xe0, 0x03, 0x00, 0x80, 0x0f, 0x1c, 0x00, 0x00, 0x00, 0xf8, 0x0f,
   0x00, 0x80, 0x03, 0x38, 0x00, 0x00, 0x00, 0x1c, 0x0c, 0x00, 0x00, 0x01,
   0x60, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x00, 0x00,
   0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x03, 0x00,
   0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
   0x80, 0x01, 0x00, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x0f, 0x00,
   0xe0, 0x10, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x1f, 0x00, 0x38, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x18, 0x78, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x60, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00,
   0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x02, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0xc0, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x80, 0x01, 0x03, 0x00, 0x00, 0x00, 0xf0, 0xff, 0x07, 0x80, 0x01,
   0x03, 0x00, 0x00, 0x00, 0xf0, 0xff, 0x03, 0x80, 0x01, 0x03, 0x00, 0x00,
   0x00, 0xf8, 0xff, 0x01, 0xc0, 0x01, 0x02, 0x00, 0x00, 0x00, 0xfc, 0xff,
   0x00, 0xc0, 0x00, 0x06, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x00, 0xe0, 0x00,
   0x06, 0x00, 0x00, 0x00, 0xff, 0x7f, 0x00, 0x70, 0x00, 0x0c, 0x00, 0x00,
   0x80, 0xff, 0x3f, 0x00, 0x38, 0x00, 0x1c, 0x00, 0xc0, 0xc1, 0xff, 0x0f,
   0x00, 0x1c, 0x00, 0x38, 0x00, 0xf8, 0xe0, 0xff, 0x07, 0x00, 0x0f, 0x00,
   0xe0, 0x03, 0xff, 0xf1, 0xff, 0x03, 0xe0, 0x07, 0x00, 0x80, 0xff, 0x87,
   0xff, 0xff, 0x83, 0xff, 0x01, 0x00, 0x00, 0x70, 0x00, 0xfc, 0xff, 0xff,
   0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xfe, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x1f, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xe0, 0xff, 0xff, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0,
   0xff, 0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x07,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xfe, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0xfe, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x1f, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xff, 0x1f, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x80, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0,
   0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xf0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8,
   0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0xfe, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x03,
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define XBM_FLASH_W 13
#define XBM_FLASH_H 14
static const unsigned char XBM_FLASH_BITS[] = {
   0x00, 0x1f, 0x80, 0x1f, 0xc0, 0x0f, 0xe0, 0x07, 0xf0, 0x03, 0xf8, 0x01,
   0xfc, 0x07, 0xe0, 0x03, 0xf0, 0x01, 0xf8, 0x00, 0x3c, 0x00, 0x0e, 0x00,
   0x07, 0x00, 0x03, 0x00 };

#define XBM_PRESS_W 11
#define XBM_PRESS_H 14
static const unsigned char XBM_PRESS_BITS[] = {
   0x04, 0x01, 0x04, 0x01, 0x04, 0x01, 0x04, 0x01, 0x55, 0x05, 0xdf, 0x07,
   0x8e, 0x03, 0x04, 0x01, 0x00, 0x00, 0x9d, 0x03, 0x62, 0x04, 0x00, 0x00,
   0x9d, 0x03, 0x62, 0x04 };

#define XBM_CLOUD_W 14
#define XBM_CLOUD_H 8
static const unsigned char XBM_CLOUD_BITS[] = {
   0xe0, 0x00, 0x10, 0x0d, 0x0c, 0x12, 0x0a, 0x21, 0x02, 0x21, 0x05, 0x20,
   0x01, 0x10, 0xfe, 0x0f };

#ifdef __cplusplus
}
#endif

#endif  // GRAPHICS_H

/* [] END OF FILE */
