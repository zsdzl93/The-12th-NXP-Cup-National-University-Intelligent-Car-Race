#include "common.h"
#include "include.h"
#include "self_include.h"

uint8 imgbuff[CAMERA_SIZE];             //定义存储接收图像的数组
uint8 img[CAMERA_H*CAMERA_W];          //解压后的二维数组

/*
   CameraGet function：The camera captures the image and displays it on the LCD
*/
void  CameraGet(void)
{
  Site_t site     = {0, 0};                           // Display the left corner of the image
  Size_t imgsize  = {CAMERA_W, CAMERA_H};             // Image size
  Size_t size     = {CAMERA_W,CAMERA_H};              // Image size in display area


    camera_get_img(); // Camera captures images
    if (bomakaiguan_8==0)
    { 
      LCD_Img_Binary_Z(site, size, imgbuff, imgsize);  // Display binarized image (scalable)
    } 
    img_extract(img,imgbuff,CAMERA_SIZE);           /*The image is decompressed and converted into a two-dimensional array*/
                                                    /*Change the image from one pixel per bit to one pixel per byte, 0xff is white*/
}

//Compressed binarized image decompression (Space for time decompression)
//srclen is the size of the binarized image
//When decompressing, there is an array inside, and the value corresponding to black and white is configured.
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {255, 0}; // Colors corresponding to 0 and 1
//    uint8_t * mdst = dst;
//    uint8_t * msrc = src;
    // Note：camera 0 is white, 1 is black
    uint8 tmpsrc;
    while(srclen --)
    {
        tmpsrc = *src++;
        *dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
    }
}
