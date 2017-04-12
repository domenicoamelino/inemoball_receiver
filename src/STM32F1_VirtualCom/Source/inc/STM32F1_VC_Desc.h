/**
  * @file    STM32F1_VC_Desc.h
  * @author  MCD Application Team & SystemsLab
  * @version V3.2.1
  * @date    09/20/2010
  * @brief   Descriptor Header for Virtual COM Port Device.
  * @details
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
  * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
  * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1_VC_DESC_H
#define __STM32F1_VC_DESC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../../../STM32F1_VC_Conf.h"

/** @defgroup STM32F1_Virtual_Com    STM32F1 Virtual Com
 * @{
 */


/** @defgroup STM32F1_VC_Descriptor    STM32F1 VC Descriptor
 * @brief Descriptor module for Virtual COM Port Device.
 * @details See the file <i>@ref STM32F1_VC_Desc.h</i> for more details.
 * @{
 */


/** @defgroup STM32F1_VC_Descriptor_Exported_Types   STM32F1 VC Descriptor Exported Types
 * @{
 */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

#define VIRTUAL_COM_PORT_DATA_SIZE              64
#define VIRTUAL_COM_PORT_INT_SIZE               8

#define VIRTUAL_COM_PORT_SIZ_DEVICE_DESC        18
#define VIRTUAL_COM_PORT_SIZ_CONFIG_DESC        67
#define VIRTUAL_COM_PORT_SIZ_STRING_LANGID      4
#define VIRTUAL_COM_PORT_SIZ_STRING_VENDOR      38
#define VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT     (2+STRING_PRODUCT_SIZE)
#define VIRTUAL_COM_PORT_SIZ_STRING_SERIAL      26

#define STANDARD_ENDPOINT_DESC_SIZE             0x09

/**
 * @}
 */


/** @defgroup STM32F1_VC_Descriptor_Exported_Constants   STM32F1 VC Descriptor Exported Constants
 * @{
 */

extern const uint8_t g_vectcVCPortDeviceDescriptor[VIRTUAL_COM_PORT_SIZ_DEVICE_DESC];
extern const uint8_t g_vectcVCPortConfigDescriptor[VIRTUAL_COM_PORT_SIZ_CONFIG_DESC];

extern const uint8_t g_vectcVCPortStringLangID[VIRTUAL_COM_PORT_SIZ_STRING_LANGID];
extern const uint8_t g_vectcVCPortStringVendor[VIRTUAL_COM_PORT_SIZ_STRING_VENDOR];
extern const uint8_t g_vectcVCPortStringProduct[VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT];
extern uint8_t g_vectcVCPortStringSerial[VIRTUAL_COM_PORT_SIZ_STRING_SERIAL];

/**
 * @}
 */


/** @defgroup STM32F1_VC_Descriptor_Exported_Macros      STM32F1 VC Descriptor Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Descriptor_Exported_Functions   STM32F1 VC Descriptor Exported Functions
 * @{
 */

/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
