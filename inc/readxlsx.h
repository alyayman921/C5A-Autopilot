#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <xlsxio_read.h>
#include <math.h>

void handling(int row,char current_cell[],double doubleed[]);
void readxlsx(const char *filename);
