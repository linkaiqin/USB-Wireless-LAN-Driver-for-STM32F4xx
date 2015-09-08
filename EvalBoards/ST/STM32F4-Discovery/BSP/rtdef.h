/*
 * File      : rtdef.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2007-01-10     Bernard      the first version
 * 2008-07-12     Bernard      remove all rt_int8, rt_uint32_t etc typedef
 * 2010-10-26     yi.qiu       add module support
 * 2010-11-10     Bernard      add cleanup callback function in thread exit.
 * 2011-05-09     Bernard      use builtin va_arg in GCC 4.x
 * 2012-11-16     Bernard      change RT_NULL from ((void*)0) to 0.
 * 2012-12-29     Bernard      change the RT_USING_MEMPOOL location and add
 *                             RT_USING_MEMHEAP condition.
 * 2012-12-30     Bernard      add more control command for graphic.
 * 2013-01-09     Bernard      change version number.
 */

#ifndef __RT_DEF_H__
#define __RT_DEF_H__

/* include rtconfig header to import configuration */


#ifdef __cplusplus
extern "C" {
#endif


/* RT-Thread basic data type definitions */
typedef signed   char                   rt_int8_t;      /**<  8bit integer type */
typedef signed   short                  rt_int16_t;     /**< 16bit integer type */
typedef signed   long                   rt_int32_t;     /**< 32bit integer type */
typedef unsigned char                   rt_uint8_t;     /**<  8bit unsigned integer type */
typedef unsigned short                  rt_uint16_t;    /**< 16bit unsigned integer type */
typedef unsigned long                   rt_uint32_t;    /**< 32bit unsigned integer type */
typedef int                             rt_bool_t;      /**< boolean type */

/* 32bit CPU */
typedef long                            rt_base_t;      /**< Nbit CPU related date type */
typedef unsigned long                   rt_ubase_t;     /**< Nbit unsigned CPU related data type */

typedef rt_base_t                       rt_err_t;       /**< Type for error number */
typedef rt_uint32_t                     rt_time_t;      /**< Type for time stamp */
typedef rt_uint32_t                     rt_tick_t;      /**< Type for tick count */
typedef rt_base_t                       rt_flag_t;      /**< Type for flags */
typedef rt_ubase_t                      rt_size_t;      /**< Type for size number */
typedef rt_ubase_t                      rt_dev_t;       /**< Type for device */
typedef rt_base_t                       rt_off_t;       /**< Type for offset */

/* boolean type definitions */
#define RT_TRUE                         1               /**< boolean true  */
#define RT_FALSE                        0               /**< boolean fails */

/*@}*/

/* maximum value of base type */
#define RT_UINT8_MAX                    0xff            /**< Maxium number of UINT8 */
#define RT_UINT16_MAX                   0xffff          /**< Maxium number of UINT16 */
#define RT_UINT32_MAX                   0xffffffff      /**< Maxium number of UINT32 */
#define RT_TICK_MAX                     RT_UINT32_MAX   /**< Maxium number of tick */


#define RT_NULL                         ((void *)0)

/**
 * @addtogroup Error
 */

/*@{*/

/* RT-Thread error code definitions */
#define RT_EOK                          0               /**< There is no error */
#define RT_ERROR                        1               /**< A generic error happens */
#define RT_ETIMEOUT                     2               /**< Timed out */
#define RT_EFULL                        3               /**< The resource is full */
#define RT_EEMPTY                       4               /**< The resource is empty */
#define RT_ENOMEM                       5               /**< No memory */
#define RT_ENOSYS                       6               /**< No system */
#define RT_EBUSY                        7               /**< Busy */
#define RT_EIO                          8               /**< IO error */

#define RT_ASSERT assert_param
#define rt_inline  static inline
#include "misc_cvt.h"


struct rt_device_blk_geometry
{
    rt_uint32_t sector_count;                       /**< count of sectors                           */
    rt_uint32_t bytes_per_sector;                   /**< number of bytes per sector                 */
    rt_uint32_t block_size;                         /**< size to erase one block                    */
};




#ifdef __cplusplus
}
#endif

#endif
