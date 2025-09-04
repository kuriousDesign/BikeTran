#ifndef GEARMAP_H
#define GEARMAP_H

#define NUM_GEARS 15

//index starts at 1, Primary then Secondary
class GearMap {
private:
    int8_t gearData[NUM_GEARS + 1][2] = {
        {0, 0}, // placeholder gear, don't use
        {1, 1}, // Gear 1
        {2, 1}, // Gear 2
        {3, 1}, // Gear 3
        {4, 1}, // Gear 4
        {5, 1}, // Gear 5
        {1, 2}, // Gear 6
        {2, 2}, // Gear 7
        {3, 2}, // Gear 8
        {4, 2}, // Gear 9
        {5, 2}, // Gear 10
        {1, 3}, // Gear 11
        {2, 3}, // Gear 12
        {3, 3}, // Gear 13
        {4, 3}, // Gear 14
        {5, 3}, // Gear 15

    };

public:
    GearMap() {
        // Initialize gearData if needed
    }

    // returns gear stage positions array [P Stage, S Stage]
    int8_t* getGearPositions(int gearNumber) {
         
        if (gearNumber >= 0 && gearNumber <= NUM_GEARS) {
            return gearData[gearNumber];
        }
        return gearData[0]; // Default for invalid gear
    }
    
};

#endif