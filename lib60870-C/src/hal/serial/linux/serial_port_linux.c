/*
 *  serial_port_linux.c
 *
 *  Copyright 2013-2024 Michael Zillgith
 *
 *  This file is part of Platform Abstraction Layer (libpal)
 *  for libiec61850, libmms, and lib60870.
 */

#include "lib_memory.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>

#include "hal_serial.h"
#include "hal_time.h"
#include "hal_socket.h"
#include <unistd.h>
#include "link_layer.h" 

struct sSerialPort {
    char interfaceName[100];
    int fd;
    int baudRate;
    uint8_t dataBits;
    char parity;
    uint8_t stopBits;
    uint64_t lastSentTime;
    struct timeval timeout;
    SerialPortError lastError;
    Socket socket;
    bool overTcp;
    int port;
    char ip[64];
    uint64_t lastConnection;
    int running;
    LinkLayerState stateSocket;
    int retry;

    
    IEC60870_LinkLayerStateChangedHandler  stateCallback;
    void* stateCallbackParameter;
};

void SerialPort_setLinkLayerStateCallback(SerialPort self, IEC60870_LinkLayerStateChangedHandler callback, void* parameter)
{
    if (self) {
        self->stateCallback = callback;
        self->stateCallbackParameter = parameter;
    }
}

bool parse_overtcp(const char* input, char* ip, int* port, int* retry)
{
    const char* prefix = "overtcp://";
    size_t prefix_len = strlen(prefix);
 
    // Verificar si la cadena comienza con "overtcp://"
    if (strncmp(input, prefix, prefix_len) != 0)
    {
        printf("Formato incorrecto\n");
        return false;
    }
 
    // Extraer la parte después del prefijo
    const char* address = input + prefix_len;
 
    // Buscar el primer separador ":"
    const char* colon_pos1 = strchr(address, ':');
    if (colon_pos1 == NULL)
    {
        printf("Formato incorrecto, falta el puerto\n");
        return false;
    }
 
    // Extraer IP
    size_t ip_len = colon_pos1 - address;
    strncpy(ip, address, ip_len);
    ip[ip_len] = '\0'; // Asegurar terminación de string

    // Buscar el segundo separador ":" para el retry
    const char* colon_pos2 = strchr(colon_pos1 + 1, ':');
    if (colon_pos2 == NULL)
    {
        // Si no hay segundo ":", asumimos retry como 10
        *port = atoi(colon_pos1 + 1);
        *retry = 10; // Valor por defecto
    }
    else
    {
        // Extraer puerto
        size_t port_len = colon_pos2 - (colon_pos1 + 1);
        char port_str[16];
        strncpy(port_str, colon_pos1 + 1, port_len);
        port_str[port_len] = '\0';
        *port = atoi(port_str);

        // Extraer retry
        *retry = atoi(colon_pos2 + 1);
    }
 
    // Extraer puerto
    //*port = atoi(colon_pos + 1);
    return true;
}


SerialPort
SerialPort_create(const char* interfaceName, int baudRate, uint8_t dataBits, char parity, uint8_t stopBits)
{
    SerialPort self = (SerialPort) GLOBAL_MALLOC(sizeof(struct sSerialPort));

    char ip[64];    
    int port; 
    int retry;
    bool overTcp = parse_overtcp(interfaceName, ip, &port, &retry);

    if (self != NULL) {
        self->fd = -1;
        self->baudRate = baudRate;
        self->dataBits = dataBits;
        self->stopBits = stopBits;
        self->parity = parity;
        self->lastSentTime = 0;
        self->lastConnection = 0;
        self->timeout.tv_sec = 0;
        self->timeout.tv_usec = 100000; /* 100 ms */
        strncpy(self->interfaceName, interfaceName, 99);
        self->lastError = SERIAL_PORT_ERROR_NONE;
        self->overTcp = overTcp;
        strncpy(self->ip, ip, 64);
        self->port = port;
        self->retry = retry;
        self->running = 0;
        
    }

    return self;
}

void
SerialPort_destroy(SerialPort self)
{
    if (self != NULL) {
        if(self->overTcp)
        {
          
            if (self->socket != NULL){              
                Socket_destroy(self->socket);
                self->socket = -1;
            }
        }
    else{
        GLOBAL_FREEMEM(self);
        }
    }
}

bool
SerialPort_open(SerialPort self)
{

    if (self->overTcp)
    {

        if (self->socket != NULL) {
            self->running = Socket_checkAsyncConnectState(self->socket);            
            Socket_destroy(self->socket);      
            self->socket = NULL;
        }
        else{
           // printf("Socket ya es NULL antes de intentar destruirlo.\n");
        }          
      
        self->lastConnection = Hal_getMonotonicTimeInMs();

        self->socket = TcpSocket_create();
        if (self->socket == NULL) {   
            if (self->stateCallback)
            self->stateCallback(self->stateCallbackParameter, 0, LL_STATE_ERROR);
            self->stateSocket = LL_STATE_ERROR;

            return false;
        }
        if (self->stateCallback)
            self->stateCallback(self->stateCallbackParameter, 0, LL_STATE_BUSY);
            self->stateSocket = LL_STATE_BUSY;

    
        if (Socket_connect(self->socket, self->ip, self->port))
        {               
            if (self->stateCallback)
                self->stateCallback(self->stateCallbackParameter, 0, LL_STATE_AVAILABLE);
                self->stateSocket = LL_STATE_AVAILABLE;
            return true;
        }           
        
        if (self->stateCallback)
        self->stateCallback(self->stateCallbackParameter, 0, LL_STATE_ERROR);
        self->stateSocket = LL_STATE_ERROR;

        Socket_destroy(self->socket);
        self->socket = NULL;           
        
        return false;           
    
    }

    self->fd = open(self->interfaceName, O_RDWR | O_NOCTTY | O_NDELAY | O_EXCL);

    if (self->fd == -1) {
        self->lastError = SERIAL_PORT_ERROR_OPEN_FAILED;
        return false;
    }

    struct termios tios;
    speed_t baudrate;

    tcgetattr(self->fd, &tios);

    switch (self->baudRate) {
    case 110:
        baudrate = B110;
        break;
    case 300:
        baudrate = B300;
        break;
    case 600:
        baudrate = B600;
        break;
    case 1200:
        baudrate = B1200;
        break;
    case 2400:
        baudrate = B2400;
        break;
    case 4800:
        baudrate = B4800;
        break;
    case 9600:
        baudrate = B9600;
        break;
    case 19200:
        baudrate = B19200;
        break;
    case 38400:
        baudrate = B38400;
        break;
    case 57600:
        baudrate = B57600;
        break;
    case 115200:
        baudrate = B115200;
        break;
    default:
        baudrate = B9600;
        self->lastError = SERIAL_PORT_ERROR_INVALID_BAUDRATE;
    }

    /* Set baud rate */
    if ((cfsetispeed(&tios, baudrate) < 0) || (cfsetospeed(&tios, baudrate) < 0)) {
        close(self->fd);
        self->fd = -1;
        self->lastError = SERIAL_PORT_ERROR_INVALID_BAUDRATE;
        return false;
    }

    tios.c_cflag |= (CREAD | CLOCAL);

    /* Set data bits (5/6/7/8) */
    tios.c_cflag &= ~CSIZE;
    switch (self->dataBits) {
    case 5:
        tios.c_cflag |= CS5;
        break;
    case 6:
        tios.c_cflag |= CS6;
        break;
    case 7:
        tios.c_cflag |= CS7;
        break;
    case 8:
    default:
        tios.c_cflag |= CS8;
        break;
    }

    /* Set stop bits (1/2) */
    if (self->stopBits == 1)
        tios.c_cflag &=~ CSTOPB;
    else /* 2 */
        tios.c_cflag |= CSTOPB;

    if (self->parity == 'N') {
        tios.c_cflag &=~ PARENB;
    } else if (self->parity == 'E') {
        tios.c_cflag |= PARENB;
        tios.c_cflag &=~ PARODD;
    } else { /* 'O' */
        tios.c_cflag |= PARENB;
        tios.c_cflag |= PARODD;
    }

    tios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    if (self->parity == 'N') {
        tios.c_iflag &= ~INPCK;
    } else {
        tios.c_iflag |= INPCK;
    }

    tios.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    tios.c_iflag |= IGNBRK; /* Set ignore break to allow 0xff characters */
    tios.c_iflag |= IGNPAR;
    tios.c_oflag &=~ OPOST;

    tios.c_cc[VMIN] = 0;
    tios.c_cc[VTIME] = 0;

    if (tcsetattr(self->fd, TCSANOW, &tios) < 0) {
        close(self->fd);
        self->fd = -1;
        self->lastError = SERIAL_PORT_ERROR_INVALID_ARGUMENT;

        return false;
    }

    return true;
}

void
SerialPort_close(SerialPort self)
{
    //printf("close\n");
     
    /*
    
    if (self != NULL) {
        if(self->overTcp)
        {
            if (self->socket != NULL){             
                Socket_destroy(self->socket);
                self->socket = -1;
            }
        }
    }    
    */
    
    if (self->fd != -1) {
        close(self->fd);
        self->fd = 0;
    }
}

int
SerialPort_getBaudRate(SerialPort self)
{
    return self->baudRate;
}

void reConnect(SerialPort self){

    if (self->stateSocket != LL_STATE_ERROR)
    { 
        return;
    }

    if (self->socket != NULL) {
        self->running = Socket_checkAsyncConnectState(self->socket);
        
     } else {
         //printf("Socket es NULL.\n");

     }        

     uint64_t delta = Hal_getMonotonicTimeInMs()- self->lastConnection; 
     int retry = self->retry * 1000; // s a ms 

     if (delta > retry){
         SerialPort_open(self);         
     }   

}

void
SerialPort_discardInBuffer(SerialPort self)
{   
    if (self->overTcp)
    {  
        self->stateSocket = LL_STATE_ERROR;
        reConnect(self);      
    }
    else{
        tcflush(self->fd, TCIOFLUSH);
    }   
    
}

void
SerialPort_setTimeout(SerialPort self, int timeout)
{
    self->timeout.tv_sec = timeout / 1000;
    self->timeout.tv_usec = (timeout % 1000) * 1000;
}

SerialPortError
SerialPort_getLastError(SerialPort self)
{
    return self->lastError;
}

int
SerialPort_readByte(SerialPort self)
{  

    uint8_t buf[1];

    if (self->overTcp)
    {
       
        if (self->socket != NULL) 
        {           
            int len = Socket_read(self->socket, buf, 1);
            if (len == 0)
                return -1;
            else
                return (int)buf[0];
        }
        
    }

    fd_set set;

    self->lastError = SERIAL_PORT_ERROR_NONE;

    FD_ZERO(&set);
    FD_SET(self->fd, &set);

    int ret = select(self->fd + 1, &set, NULL, NULL, &(self->timeout));

    if (ret == -1) {
        self->lastError = SERIAL_PORT_ERROR_UNKNOWN;
        return -1;
    }
    else if (ret == 0)
        return -1;
    else {
        read(self->fd, (char*) buf, 1);

        return (int) buf[0];
    }
}

int
SerialPort_write(SerialPort self, uint8_t* buffer, int startPos, int bufSize)
{
    /* TODO assure minimum line idle time? */

    if (self->overTcp)
    {
        if (self->socket != NULL) 
        {       
            return Socket_write(self->socket, buffer + startPos, bufSize);    
        }   
        reConnect(self); 
        return  -1;               
    }

    self->lastError = SERIAL_PORT_ERROR_NONE;

    ssize_t result = write(self->fd, buffer + startPos, bufSize);

    tcdrain(self->fd);

    self->lastSentTime = Hal_getMonotonicTimeInMs();

    return result;
}
