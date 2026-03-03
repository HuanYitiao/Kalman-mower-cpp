#include "data_generator.h"

int main()
{
    DataGenerator dataGenerator(0.01);
    dataGenerator.generate();
    dataGenerator.saveCSV("data/output.csv");
}