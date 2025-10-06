// link_layer.c - Corrected reliable version with RR/REJ acknowledgments
#include "link_layer.h"
#include "serial_port.h"
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>

#define FLAG 0x7E
#define A_SENDER 0x03
#define A_RECEIVER 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define C_RR0 0x05
#define C_RR1 0x85
#define C_REJ0 0x01
#define C_REJ1 0x81

#define MAX_RETRIES 3
#define TIMEOUT 3 // seconds

static int fd = -1;
static volatile sig_atomic_t alarmFlag = 0;
static int sequenceNumber = 0; // sequence of next I-frame to send

void handleAlarm(int sig) {
    (void)sig; // Suppress unused parameter warning
    alarmFlag = 1;
}

// -------------------- BYTE-STUFFING --------------------
void stuffData(const unsigned char *data, int size, unsigned char *dest, int *destSize)
{
    int j = 0;
    for (int i = 0; i < size; i++)
    {
        if (data[i] == FLAG) {
            dest[j++] = 0x7D;
            dest[j++] = 0x5E;
        }
        else if (data[i] == 0x7D) {
            dest[j++] = 0x7D;
            dest[j++] = 0x5D;
        }
        else {
            dest[j++] = data[i];
        }
    }
    *destSize = j;
}

void destuffData(const unsigned char *data, int size, unsigned char *dest, int *destSize)
{
    int j = 0;
    for (int i = 0; i < size; i++)
    {
        if (data[i] == 0x7D)
        {
            i++;
            if (i < size) {
                dest[j++] = (data[i] == 0x5E) ? FLAG : 0x7D;
            }
        }
        else {
            dest[j++] = data[i];
        }
    }
    *destSize = j;
}

// -------------------- LLOPEN --------------------
int llopen(LinkLayer connectionParameters)
{
    fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);
    if (fd < 0) {
        printf("Error: Failed to open serial port\n");
        return -1;
    }

    signal(SIGALRM, handleAlarm);
    unsigned char frame[5];
    unsigned char recvByte;
    int retries = 0;

    if (connectionParameters.role == LlTx)
    {
        // Build SET frame
        frame[0] = FLAG;
        frame[1] = A_SENDER;
        frame[2] = C_SET;
        frame[3] = frame[1] ^ frame[2];
        frame[4] = FLAG;

        while (retries < MAX_RETRIES)
        {
            printf("Sending SET frame (attempt %d/%d)...\n", retries + 1, MAX_RETRIES);
            writeBytesSerialPort(frame, 5);
            alarm(TIMEOUT);
            alarmFlag = 0;

            unsigned char state = 0;
            while (!alarmFlag)
            {
                if (readByteSerialPort(&recvByte) <= 0) continue;
                switch (state)
                {
                case 0:
                    if (recvByte == FLAG) state = 1;
                    break;
                case 1:
                    if (recvByte == A_RECEIVER) state = 2;
                    else if (recvByte != FLAG) state = 0;
                    break;
                case 2:
                    if (recvByte == C_UA) state = 3;
                    else if (recvByte == FLAG) state = 1;
                    else state = 0;
                    break;
                case 3:
                    if (recvByte == (A_RECEIVER ^ C_UA)) state = 4;
                    else if (recvByte == FLAG) state = 1;
                    else state = 0;
                    break;
                case 4:
                    if (recvByte == FLAG) {
                        alarm(0);
                        printf("Connection established (UA received)\n");
                        return fd;
                    }
                    else state = 0;
                    break;
                }
            }
            retries++;
            printf("Timeout! No UA received.\n");
        }

        printf("Error: Failed to establish connection after %d retries\n", MAX_RETRIES);
        alarm(0);
        closeSerialPort();
        return -1;
    }
    else // RECEIVER
    {
        printf("Waiting for SET frame...\n");
        unsigned char state = 0;
        while (1)
        {
            if (readByteSerialPort(&recvByte) <= 0) continue;
            switch (state)
            {
            case 0:
                if (recvByte == FLAG) state = 1;
                break;
            case 1:
                if (recvByte == A_SENDER) state = 2;
                else if (recvByte != FLAG) state = 0;
                break;
            case 2:
                if (recvByte == C_SET) state = 3;
                else if (recvByte == FLAG) state = 1;
                else state = 0;
                break;
            case 3:
                if (recvByte == (A_SENDER ^ C_SET)) state = 4;
                else if (recvByte == FLAG) state = 1;
                else state = 0;
                break;
            case 4:
                if (recvByte == FLAG)
                {
                    // Send UA
                    frame[0] = FLAG;
                    frame[1] = A_RECEIVER;
                    frame[2] = C_UA;
                    frame[3] = frame[1] ^ frame[2];
                    frame[4] = FLAG;
                    writeBytesSerialPort(frame, 5);
                    printf("Connection established (UA sent)\n");
                    return fd;
                }
                else state = 0;
                break;
            }
        }
    }
}

// -------------------- LLWRITE --------------------
int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char frame[2 * bufSize + 6];
    int retries = 0;

    while (retries < MAX_RETRIES)
    {
        // Build I-frame
        unsigned char control = (sequenceNumber == 0) ? 0x00 : 0x40;
        unsigned char dataWithBCC[bufSize + 1];
        memcpy(dataWithBCC, buf, bufSize);

        // Calculate BCC2
        unsigned char bcc2 = 0;
        for (int i = 0; i < bufSize; i++) {
            bcc2 ^= buf[i];
        }
        dataWithBCC[bufSize] = bcc2;

        // Stuff data + BCC2
        int stuffedSize;
        stuffData(dataWithBCC, bufSize + 1, frame + 4, &stuffedSize);

        // Build frame header
        frame[0] = FLAG;
        frame[1] = A_SENDER;
        frame[2] = control;
        frame[3] = frame[1] ^ frame[2];
        frame[4 + stuffedSize] = FLAG;
        int totalSize = 5 + stuffedSize;

        // Send frame
        printf("Sending I-frame (seq=%d, attempt %d/%d)...\n", sequenceNumber, retries + 1, MAX_RETRIES);
        writeBytesSerialPort(frame, totalSize);

        // Wait for RR/REJ
        alarm(TIMEOUT);
        alarmFlag = 0;

        unsigned char byte;
        unsigned char state = 0;
        unsigned char receivedControl = 0;
        int ackReceived = 0;

        while (!alarmFlag && !ackReceived)
        {
            if (readByteSerialPort(&byte) <= 0) continue;

            switch (state)
            {
            case 0:
                if (byte == FLAG) state = 1;
                break;
            case 1:
                if (byte == A_RECEIVER) state = 2;
                else if (byte != FLAG) state = 0;
                break;
            case 2:
                receivedControl = byte;
                // Expecting RR for NEXT sequence (if sent seq=0, expect RR1)
                if (byte == ((sequenceNumber == 0) ? C_RR1 : C_RR0)) {
                    state = 3;
                }
                else if (byte == ((sequenceNumber == 0) ? C_REJ0 : C_REJ1)) {
                    state = 3;
                }
                else if (byte == FLAG) {
                    state = 1;
                }
                else {
                    state = 0;
                }
                break;
            case 3:
                if (byte == (A_RECEIVER ^ receivedControl)) {
                    state = 4;
                }
                else if (byte == FLAG) {
                    state = 1;
                }
                else {
                    state = 0;
                }
                break;
            case 4:
                if (byte == FLAG) {
                    ackReceived = 1;
                }
                else {
                    state = 0;
                }
                break;
            }
        }

        alarm(0); // Cancel alarm

        if (ackReceived)
        {
            // Check if it was RR or REJ
            if (receivedControl == ((sequenceNumber == 0) ? C_RR1 : C_RR0))
            {
                printf("RR received, frame accepted\n");
                sequenceNumber ^= 1; // Toggle sequence number
                return bufSize;
            }
            else // REJ received
            {
                printf("REJ received, retransmitting frame...\n");
                retries++;
            }
        }
        else
        {
            printf("Timeout! No acknowledgment received.\n");
            retries++;
        }
    }

    printf("Error: Failed to send frame after %d retries\n", MAX_RETRIES);
    return -1;
}

// -------------------- LLREAD --------------------
int llread(unsigned char *packet)
{
    unsigned char byte;
    unsigned char frame[2048];
    int idx = 0;
    int inFrame = 0;
    static int expectedSeq = 0; // Track expected sequence number

    while (1)
    {
        if (readByteSerialPort(&byte) <= 0) continue;

        if (byte == FLAG)
        {
            if (!inFrame) {
                // Start of new frame
                inFrame = 1;
                idx = 0;
                frame[idx++] = byte;
            }
            else {
                // End of frame
                frame[idx++] = byte;
                break;
            }
        }
        else if (inFrame) {
            if (idx < 2048) {
                frame[idx++] = byte;
            }
            else {
                // Frame too large, reset
                inFrame = 0;
                idx = 0;
            }
        }
    }

    // Validate frame length (minimum 5 bytes)
    if (idx < 5) {
        printf("Frame too short\n");
        goto send_rej;
    }

    // Check BCC1
    if ((frame[1] ^ frame[2]) != frame[3])
    {
        printf("BCC1 error detected\n");
        goto send_rej;
    }

    // Extract control byte and check sequence
    unsigned char control = frame[2];
    int receivedSeq = (control & 0x40) ? 1 : 0;

    // Check if this is a duplicate frame
    if (receivedSeq != expectedSeq)
    {
        printf("Duplicate frame detected (seq=%d, expected=%d), sending RR\n", receivedSeq, expectedSeq);
        // Send RR for next expected frame (don't change expectedSeq)
        unsigned char rr = (expectedSeq == 0) ? C_RR1 : C_RR0;
        unsigned char rrFrame[5] = {FLAG, A_RECEIVER, rr, A_RECEIVER ^ rr, FLAG};
        writeBytesSerialPort(rrFrame, 5);
        return -1; // Don't pass duplicate to application
    }

    // Destuff data
    unsigned char data[2048];
    int dataSize;
    destuffData(frame + 4, idx - 5, data, &dataSize);

    // Validate we have at least BCC2
    if (dataSize < 1) {
        printf("No data in frame\n");
        goto send_rej;
    }

    // Check BCC2
    unsigned char bcc2 = 0;
    for (int i = 0; i < dataSize - 1; i++) {
        bcc2 ^= data[i];
    }

    if (bcc2 != data[dataSize - 1])
    {
        printf("BCC2 error detected\n");
        goto send_rej;
    }

    // Frame is valid, send RR for NEXT sequence
    unsigned char rr = (expectedSeq == 0) ? C_RR1 : C_RR0;
    unsigned char rrFrame[5] = {FLAG, A_RECEIVER, rr, A_RECEIVER ^ rr, FLAG};
    writeBytesSerialPort(rrFrame, 5);
    printf("Frame accepted (seq=%d), RR sent\n", receivedSeq);

    // Copy data (without BCC2) to packet
    memcpy(packet, data, dataSize - 1);
    expectedSeq ^= 1; // Toggle expected sequence
    return dataSize - 1;

send_rej:
    // Send REJ for current expected sequence
    unsigned char rej = (expectedSeq == 0) ? C_REJ0 : C_REJ1;
    unsigned char rejFrame[5] = {FLAG, A_RECEIVER, rej, A_RECEIVER ^ rej, FLAG};
    writeBytesSerialPort(rejFrame, 5);
    printf("REJ sent (expecting seq=%d)\n", expectedSeq);
    return -1;
}

// -------------------- LLCLOSE --------------------
int llclose(int showRole)
{
    unsigned char frame[5];
    unsigned char recvByte;
    int retries = 0;

    if (showRole == LlTx)
    {
        // TRANSMITTER: Send DISC, wait for DISC, send UA
        while (retries < MAX_RETRIES)
        {
            // Send DISC
            frame[0] = FLAG;
            frame[1] = A_SENDER;
            frame[2] = C_DISC;
            frame[3] = frame[1] ^ frame[2];
            frame[4] = FLAG;

            printf("Sending DISC (attempt %d/%d)...\n", retries + 1, MAX_RETRIES);
            writeBytesSerialPort(frame, 5);

            // Wait for DISC from receiver
            alarm(TIMEOUT);
            alarmFlag = 0;

            unsigned char state = 0;
            while (!alarmFlag)
            {
                if (readByteSerialPort(&recvByte) <= 0) continue;

                switch (state)
                {
                case 0: if (recvByte == FLAG) state = 1; break;
                case 1:
                    if (recvByte == A_RECEIVER) state = 2;
                    else if (recvByte != FLAG) state = 0;
                    break;
                case 2:
                    if (recvByte == C_DISC) state = 3;
                    else if (recvByte == FLAG) state = 1;
                    else state = 0;
                    break;
                case 3:
                    if (recvByte == (A_RECEIVER ^ C_DISC)) state = 4;
                    else if (recvByte == FLAG) state = 1;
                    else state = 0;
                    break;
                case 4:
                    if (recvByte == FLAG) goto disc_received;
                    else state = 0;
                    break;
                }
            }

            retries++;
            printf("Timeout! No DISC received.\n");
        }

        printf("Error: Failed to receive DISC after %d retries\n", MAX_RETRIES);
        alarm(0);
        closeSerialPort();
        return -1;

disc_received:
        alarm(0);
        printf("DISC received from receiver\n");

        // Send UA
        frame[0] = FLAG;
        frame[1] = A_RECEIVER;
        frame[2] = C_UA;
        frame[3] = frame[1] ^ frame[2];
        frame[4] = FLAG;
        writeBytesSerialPort(frame, 5);
        printf("UA sent, connection closed\n");

        sleep(1); // Give time for UA to be sent
        closeSerialPort();
        return 0;
    }
    else // RECEIVER
    {
        // RECEIVER: Wait for DISC, send DISC, wait for UA
        printf("Waiting for DISC from transmitter...\n");

        unsigned char state = 0;
        while (1)
        {
            if (readByteSerialPort(&recvByte) <= 0) continue;

            switch (state)
            {
            case 0: if (recvByte == FLAG) state = 1; break;
            case 1:
                if (recvByte == A_SENDER) state = 2;
                else if (recvByte != FLAG) state = 0;
                break;
            case 2:
                if (recvByte == C_DISC) state = 3;
                else if (recvByte == FLAG) state = 1;
                else state = 0;
                break;
            case 3:
                if (recvByte == (A_SENDER ^ C_DISC)) state = 4;
                else if (recvByte == FLAG) state = 1;
                else state = 0;
                break;
            case 4:
                if (recvByte == FLAG) goto send_disc;
                else state = 0;
                break;
            }
        }

send_disc:
        printf("DISC received from transmitter\n");

        // Send DISC
        frame[0] = FLAG;
        frame[1] = A_RECEIVER;
        frame[2] = C_DISC;
        frame[3] = frame[1] ^ frame[2];
        frame[4] = FLAG;
        writeBytesSerialPort(frame, 5);
        printf("DISC sent\n");

        // Wait for UA
        state = 0;
        alarm(TIMEOUT * 2); // Give more time for final UA
        alarmFlag = 0;

        while (!alarmFlag)
        {
            if (readByteSerialPort(&recvByte) <= 0) continue;

            switch (state)
            {
            case 0: if (recvByte == FLAG) state = 1; break;
            case 1:
                if (recvByte == A_RECEIVER) state = 2;
                else if (recvByte != FLAG) state = 0;
                break;
            case 2:
                if (recvByte == C_UA) state = 3;
                else if (recvByte == FLAG) state = 1;
                else state = 0;
                break;
            case 3:
                if (recvByte == (A_RECEIVER ^ C_UA)) state = 4;
                else if (recvByte == FLAG) state = 1;
                else state = 0;
                break;
            case 4:
                if (recvByte == FLAG) {
                    alarm(0);
                    printf("UA received, connection closed\n");
                    closeSerialPort();
                    return 0;
                }
                else state = 0;
                break;
            }
        }

        // Timeout waiting for UA, but still close
        alarm(0);
        printf("Timeout waiting for UA, closing anyway\n");
        closeSerialPort();
        return 0;
    }
}