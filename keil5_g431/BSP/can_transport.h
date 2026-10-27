#ifndef CAN_TRANSPORT_H
#define CAN_TRANSPORT_H
int can_transport_init(void);
void can_transport_service(void);
unsigned long can_transport_dropped_bytes(void);
#endif
