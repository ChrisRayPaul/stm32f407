/*
 * File      : queue.h
 * This file is part of heluo
 * COPYRIGHT (C) 2017, aistlab Development Team
 *
 * Change Logs:
 * Date           Author       Notes
 *2018-05-30	  ChrisRay	   create file
 */

/*!
 * @brief
 * @param
 * @return
 */
#define QUEUE_MAX_SIZE	32

struct queue_tag{
	char buff[QUEUE_MAX_SIZE];
	uint8_t head;
	uint8_t tail;

}Queue;
