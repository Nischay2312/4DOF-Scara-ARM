/*
    Author: Nischay Joshi
    Date:   March, 31, 2022
    Notes:  Loads BaseMotorAngles 
*/

#ifndef _BASEANGLE_H    
#define _BASEANGLE_H

const int8_t PROGMEM BA[] = {
0,0,0,0,0,1,1,2,3,4,5,6,7,9,10,12,14,15,17,18,19,20,21,22,22,23,24,25,26,27,28,30,31,32,34,35,36,37,38,38,39,40,41,41,42,43,44,45,47,48,49,50,52,53,54,56,57,58,59,60,61,62,63,64,65,66,68,69,70,72,73,74,75,76,77,78,79,80,81,81,83,84,85,86,87,88,89,90,91,91,92,93,93,94,94,95,95,96,97,97,98,99,99,100,100,100,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,101,102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,102,103,103,103,103,102,102,102,102,102,102,102,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,103,104,104,104,104,104,103,103,103,103,103,103,104,104,104,104,104,104,104,104,104,104,104,104,104,104,104,104,104,105,105,105,105,105,105,105,104,104,104,104,104,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,105,106,106,106,106,106,106,106,105,105,105,105,105,106,106,106,106,106,106,106,106,106,106,106,106,106,106,106,106,106,107,107,107,107,107,107,106,106,106,106,106,106,107,107,107,107,107,107,107,107,107,107,107,107,107,107,107,107,107,107,108,108,108,108,108,107,107,107,107,107,107,107,108,108,108,108,108,108,108,108,108,108,108,108,108,108,108,108,108,108,109,109,108,108,108,108,108,108,108,108,108,109,109,109,109,109,109,109,109,109,108,109,109,109,109,109,109,109,109,109,109,109,109,109,109,109,109,109,109,109,110,110,110,110,110,110,109,109,109,109,109,109,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,110,111,111,111,111,111,111,111,110,110,110,110,110,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,111,112,112,112,112,112,112,111,111,111,111,111,111,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,113,113,113,112,112,112,112,112,112,112,112,112,113,113,113,113,113,113,113,113,112,112,112,112,112,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,113,114,114,114,114,114,113,113,113,113,113,113,113,113,114,114,114,114,114,114,114,113,113,113,113,113,113,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,114,115,115,115,115,115,114,114,114,114,114,114,114,114,115,115,115,115,115,115,115,114,114,114,114,114,114,115,115,115,115,115,115,115,115,115,115,114,114,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,115,116,116,116,116,116,115,115,115,115,115,115,115,115,116,116,116,116,116,116,116,115,115,115,115,115,115,116,116,116,116,116,116,116,116,116,115,115,115,115,116,116,116,116,116,116,116,116,116,116,116,115,115,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,116,117,117,116,116,116,116,116,116,116,116,116,116,116,117,117,117,117,117,116,116,116,116,116,116,116,116,117,117,117,117,117,117,116,116,116,116,116,116,116,116,117,117,117,117,117,117,117,116,116,116,116,116,116,117,117,117,117,117,117,117,117,116,116,116,116,116,116,117,117,117,117,117,117,117,117,117,116,116,116,116,117,117,117,117,117,117,117,117,117,117,117,116,116,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,117,118,117,117,117,117,117,117,117,117,117,117,117,117,118,118,118,117,117,117,117,117,117,117,117,117,117,118,118,118,118,117,117,117,117,117,117,117,117,117,117,118,118,118,118,118,117,117,117,117,117,117,117,117,117,118,118,118,118,118,117,117,117,117,117,117,117,117,118,118,118,118,118,118,117,117,117,117,117,117,117,117,118,118,118,118,118,118,118,117,117,117,117,117,117,117,118,118,118,118,118,118,118,117,117,117,117,117,117,117,118,118,118,118,118,118,118,117,117,117,117,117,117,117,118,118,118,118,118,118,118,117,117,117,117,117,117,118,118,118,118,118,118,118,118,117,117,117,117,117,117,118,118,118,118,118,118,118,118,117,117,117,117,117,117,118,118,118,118,118,118,118,118,117,117,117,117,117,117,118,118,118,118,118,118,118,118,118,117,117,117,117,117,118,118,118,118,118,118,118,118,118,117,117,117,117,117,118,118,118,118,118,118,118,118,118,117,117,117,117,117,118,118,118,118,118,118,118,118,118,117,117,117,117,117,118,118,118,118,118,118,118,118,118,117,117,117,117,118,118,118,118,118,118,118,118,118,118,117,117,117,117,118,118,118,118,118,118,118,118,118,118,117,117,117,117,118,118,118,118,118,118,118,118,118,118,117,117,117,117,118,118,118,118,118,118,118,118,118,118,117,117,117,117,118,118,118,118,118,118,118,118,118,118,117,117,117,117,118,118,118,118,118,118,118,118,118,118,117,117,117,117,118,118,118,118,118,118,118,118,118,118,117,117,117,117,118,118,118,118,118,118,118,118,118,118,117,117,117,117,118,118,118,118,118,118,118,118,118,118,118,117,117,117,118,118,118,118,118,118,118,118,118,118,118,117,117,117,118,118,118,118,118,118,118,118,118,118,118,117,117,117,118,118,118,118,118,118,118,118,118,118,118,117,117,118,118,118,118,118,118,118,118,118,118,118,118,117,117,118,118,118,118,118,118,118,118,118,118,118,118,117,117,118,118,118,118,118,118,118,118,118,118,118,118,117,117,118,118,118,118,118,118,118,118,118,118,118,118,117,117,118,118,118,118,118,118,118,118,118,118,118,118,118,117,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118,118

};
#endif
