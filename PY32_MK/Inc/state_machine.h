/*
*   File: state_machine.h
*   Project: NAATOS
*   Copyright 2025, Global Health Labs
*/

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

enum state_machine
{
    low_power = 0,
    stage1,
    stage2,
    stage3,
    detection,
    self_test_1,
    self_test_2,
    preheat
};

#endif /* STATE_MACHINE_H_*/