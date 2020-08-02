/*
 * @file boat.h
 */

#ifndef BOAT_H
#define BOAT_H

/******************************************************************************
 * @brief Init Boat
 ******************************************************************************/
void Boat_Init(void);

/******************************************************************************
 * @brief Gather Boat's input data
 ******************************************************************************/
void Boat_Input(void);

/******************************************************************************
 * @brief Process Boat's input data
 ******************************************************************************/
void Boat_Process(void);

/******************************************************************************
 * @brief Output to Boat's peripherals
 ******************************************************************************/
void Boat_Output(void);

#endif /* !BOAT_H */
