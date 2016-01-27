/******************************************************************************
* File: Lamppost.h                                           Garduino Library *
* Description: Header to give utility functions in the project.               *
*                                                                             *
* Copyright (c) 2016 At the Lamppost. All rights reserved.                    *
* Written by Trey Holdener <trey@atthelamppost.net>                           *
* MIT License (see LICENSE.txt)                                               *
******************************************************************************/

#ifndef __LAMPPOST_H__

#define __LAMPPOST_H__

#define ADD_BIT(flag, bit)      ((flag) |= (bit))
#define REMOVE_BIT(flag, bit)   ((flag) &= ~(bit))
#define HAS_BIT(flag, bit)      ((flag) & (bit))

#endif
