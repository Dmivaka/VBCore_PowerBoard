void FDCAN_RxProcessing( uint32_t id, uint8_t data_size, uint8_t *data);
void process_canard_TX_queue(void);

void uavcan_setup(void);
void uavcan_spin(void);