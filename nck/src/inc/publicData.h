/*****************************************************************//**
 * \file   publicData.h
 * \brief  公用数据声明，通过PubDef宏实现。对各个模块的自定义数据类型汇集在这里，外部调用算法模块和编写测试用例的时候只需要包含该文件即可。
 * 通过定义宏实现条件编译，实际定义数据的位置是在包含PUBDEF宏定义的文件中，这里是publicData.c。
 * 只需包含该头文件publicData.h 就可以把对应的数据的作用域拓展到对应的位置。
 * 
 * \author galaxy
 * \date   March 2023
 *********************************************************************/

#ifndef __PUBLICData_H__
#define __PUBLICData_H__
#ifdef __cplusplus
extern "C" {
#endif
//宏定义
#define deg2rad			0.017453292519943
#define rad2deg			57.295779513082323
#define debugLog if(debugEnable) printf
#define ErrLog	if(errEnable) printf

//通过条件编译，实现公用数据集中定义和extern集中在publicData.h中声明。
#ifdef PUBDEF
#define PubDef 
#else
#define PubDef extern 
#endif

#include<math.h>
#include<string.h>
#include<stdio.h>
//公共数据声明
#include"velocityProfile.h"

//公共数据声明
	PubDef int debugEnable;
	PubDef int errEnable;

#ifdef __cplusplus
}
#endif
#endif