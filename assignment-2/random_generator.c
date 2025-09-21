#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Here is the ready random number generator function for you
// You can add it before your main() in the project
int generateRandomNumber(int min, int max) {
    int range_size = max - min + 1;
    return min + (rand() % range_size);
}

// Here is the example usage
int main() {
    srand(time(NULL)); // Seed the random number generator once

    int randomNumber1 = generateRandomNumber(1, 10); // Random number between 1 and 10
    printf("Random number between 1 and 10: %d\n", randomNumber1);

    int randomNumber2 = generateRandomNumber(50, 100); // Random number between 50 and 100
    printf("Random number between 50 and 100: %d\n", randomNumber2);

    return 0;
}