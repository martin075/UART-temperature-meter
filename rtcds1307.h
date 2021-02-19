uint8_t i2crtc_start(char write_address);
uint8_t i2crtc_repeated_start(char read_address);
void i2crtc_stop();
void i2crtc_start_wait(char write_address);
uint8_t i2crtc_write(char data);
char i2crtc_read_ack();
char i2crtc_read_nack();
void _ds1307_start_oscillator();






