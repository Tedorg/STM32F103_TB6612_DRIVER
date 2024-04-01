#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>

typedef struct encoder{
    volatile uint32_t *timer;
    uint16_t current_count;
    int32_t position;
    float speed; // units per minute
    
}  encoder_t;

// Function declarations or any other code you want to include in the header file
uint8_t init_encoder(encoder_t *enc, uint8_t number_of_encoders);
// float calculate_speed( int encoder_count, int time);

// void init_encoder( encoder_t* left, encoder_t* right) {
void update_encoder(encoder_t *enc);
// float get_speed(struct encoder* enc);

float get_speed_enc(encoder_t *enc);

int32_t get_position_enc(encoder_t *enc);

void reset_position_enc(encoder_t *enc);

void print_debug_info(encoder_t *enc);


#endif // ENCODER_H