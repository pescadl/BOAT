/*
 * @file ldr.h
 */

#ifndef LDR_H
#define LDR_H

typedef enum ambience
{
    AMBIENCE_BRIGHT,
    AMBIENCE_DARK
} ambience_t;

void LDR_Init();

ambience_t LDR_getAmbientLight();

void LDR_getState();


#endif /* !LDR_H */
