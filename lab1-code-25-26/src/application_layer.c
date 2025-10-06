// Application layer protocol implementation
#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <libgen.h>

// Control field values
#define CTRL_DATA 0x01
#define CTRL_START 0x02
#define CTRL_END 0x03

// TLV Types
#define TLV_FILE_SIZE 0x00
#define TLV_FILE_NAME 0x01

#define MAX_PAYLOAD_SIZE 256

// -------------------- HELPER FUNCTIONS --------------------

/**
 * Creates a control packet (START or END)
 * @param controlField CTRL_START or CTRL_END
 * @param filename Name of the file
 * @param fileSize Size of the file in bytes
 * @param packet Output buffer for the packet
 * @return Size of the control packet
 */
int buildControlPacket(unsigned char controlField, const char *filename,
                       long fileSize, unsigned char *packet)
{
    int idx = 0;

    // Control field
    packet[idx++] = controlField;

    // TLV for file size
    packet[idx++] = TLV_FILE_SIZE;

    // Calculate number of bytes needed for file size
    unsigned char sizeBytes[8];
    int numSizeBytes = 0;
    long tempSize = fileSize;

    do {
        sizeBytes[numSizeBytes++] = tempSize & 0xFF;
        tempSize >>= 8;
    } while (tempSize > 0);

    packet[idx++] = numSizeBytes; // Length of file size

    // Write file size (little-endian)
    for (int i = numSizeBytes - 1; i >= 0; i--) {
        packet[idx++] = sizeBytes[i];
    }

    // TLV for filename
    packet[idx++] = TLV_FILE_NAME;
    int nameLen = strlen(filename);
    if (nameLen > 255) nameLen = 255; // Limit filename length
    packet[idx++] = nameLen;
    memcpy(&packet[idx], filename, nameLen);
    idx += nameLen;

    return idx;
}

/**
 * Parses a control packet to extract file information
 * @param packet The received control packet
 * @param packetSize Size of the packet
 * @param fileSize Output: extracted file size
 * @param filename Output: extracted filename
 * @return 0 on success, -1 on error
 */
int parseControlPacket(const unsigned char *packet, int packetSize,
                       long *fileSize, char *filename)
{
    int idx = 1; // Skip control field
    *fileSize = 0;
    filename[0] = '\0';

    while (idx < packetSize) {
        unsigned char type = packet[idx++];
        unsigned char length = packet[idx++];

        if (idx + length > packetSize) {
            printf("Error: Malformed control packet\n");
            return -1;
        }

        if (type == TLV_FILE_SIZE) {
            // Parse file size (big-endian)
            for (int i = 0; i < length; i++) {
                *fileSize = (*fileSize << 8) | packet[idx++];
            }
        }
        else if (type == TLV_FILE_NAME) {
            memcpy(filename, &packet[idx], length);
            filename[length] = '\0';
            idx += length;
        }
        else {
            // Unknown TLV, skip it
            idx += length;
        }
    }

    return 0;
}

/**
 * Creates a data packet
 * @param sequenceNum Sequence number (0-255, wraps around)
 * @param data Data to send
 * @param dataSize Size of data
 * @param packet Output buffer for the packet
 * @return Size of the data packet
 */
int buildDataPacket(unsigned char sequenceNum, const unsigned char *data,
                    int dataSize, unsigned char *packet)
{
    int idx = 0;

    packet[idx++] = CTRL_DATA;
    packet[idx++] = sequenceNum;
    packet[idx++] = (dataSize >> 8) & 0xFF; // L2 (high byte)
    packet[idx++] = dataSize & 0xFF;        // L1 (low byte)

    memcpy(&packet[idx], data, dataSize);
    idx += dataSize;

    return idx;
}

// -------------------- TRANSMITTER --------------------

/**
 * Transmits a file over the serial port
 */
int transmitFile(LinkLayer *ll, const char *filename)
{
    // Open file
    FILE *file = fopen(filename, "rb");
    if (!file) {
        printf("Error: Cannot open file '%s'\n", filename);
        return -1;
    }

    // Get file size
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    printf("File to send: %s (%ld bytes)\n", filename, fileSize);

    // Extract just the filename (without path)
    char *baseFilename = basename((char *)filename);

    // Build and send START control packet
    unsigned char controlPacket[512];
    int controlSize = buildControlPacket(CTRL_START, baseFilename, fileSize, controlPacket);

    printf("Sending START control packet...\n");
    if (llwrite(controlPacket, controlSize) < 0) {
        printf("Error: Failed to send START packet\n");
        fclose(file);
        return -1;
    }

    // Send data packets
    unsigned char buffer[MAX_PAYLOAD_SIZE];
    unsigned char dataPacket[MAX_PAYLOAD_SIZE + 4];
    unsigned char sequenceNum = 0;
    int bytesRead;
    long totalSent = 0;
    int packetCount = 0;

    printf("Sending data packets...\n");
    while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
        int packetSize = buildDataPacket(sequenceNum, buffer, bytesRead, dataPacket);

        if (llwrite(dataPacket, packetSize) < 0) {
            printf("Error: Failed to send data packet %d\n", sequenceNum);
            fclose(file);
            return -1;
        }

        totalSent += bytesRead;
        packetCount++;
        sequenceNum = (sequenceNum + 1) % 256; // Wrap around at 256

        // Progress indicator
        if (packetCount % 10 == 0 || totalSent == fileSize) {
            printf("Progress: %ld/%ld bytes (%.1f%%)\n",
                   totalSent, fileSize, (totalSent * 100.0) / fileSize);
        }
    }

    fclose(file);
    printf("Data transmission complete: %d packets, %ld bytes\n", packetCount, totalSent);

    // Build and send END control packet
    controlSize = buildControlPacket(CTRL_END, baseFilename, fileSize, controlPacket);

    printf("Sending END control packet...\n");
    if (llwrite(controlPacket, controlSize) < 0) {
        printf("Error: Failed to send END packet\n");
        return -1;
    }

    printf("File transfer successful!\n");
    return 0;
}

// -------------------- RECEIVER --------------------

/**
 * Receives a file over the serial port
 */
int receiveFile(LinkLayer *ll, const char *filename)
{
    unsigned char packet[MAX_PAYLOAD_SIZE * 2];
    int packetSize;
    long expectedFileSize = 0;
    long totalReceived = 0;
    int packetCount = 0;
    char receivedFilename[256];
    FILE *file = NULL;

    printf("Waiting for START control packet...\n");

    // Wait for START control packet
    while (1) {
        packetSize = llread(packet);

        if (packetSize < 0) {
            continue; // Error reading, retry
        }

        if (packetSize > 0 && packet[0] == CTRL_START) {
            printf("START packet received\n");

            if (parseControlPacket(packet, packetSize, &expectedFileSize, receivedFilename) < 0) {
                printf("Error: Failed to parse START packet\n");
                return -1;
            }

            printf("File info - Name: %s, Size: %ld bytes\n", receivedFilename, expectedFileSize);
            break;
        }
    }

    // Open output file
    file = fopen(filename, "wb");
    if (!file) {
        printf("Error: Cannot create file '%s'\n", filename);
        return -1;
    }

    printf("Receiving data packets...\n");

    // Receive data packets
    unsigned char expectedSeq = 0;
    while (1) {
        packetSize = llread(packet);

        if (packetSize < 0) {
            continue; // Error reading, retry
        }

        if (packetSize == 0) {
            continue; // Empty packet
        }

        unsigned char controlField = packet[0];

        if (controlField == CTRL_END) {
            printf("END packet received\n");

            // Verify file size
            long receivedFileSize = 0;
            char endFilename[256];
            parseControlPacket(packet, packetSize, &receivedFileSize, endFilename);

            if (receivedFileSize != expectedFileSize) {
                printf("Warning: File size mismatch (expected: %ld, received: %ld)\n",
                       expectedFileSize, receivedFileSize);
            }

            if (totalReceived != expectedFileSize) {
                printf("Warning: Data size mismatch (expected: %ld, got: %ld)\n",
                       expectedFileSize, totalReceived);
            }

            break;
        }
        else if (controlField == CTRL_DATA) {
            // Parse data packet
            unsigned char sequenceNum = packet[1];
            int dataLength = (packet[2] << 8) | packet[3];

            // Check sequence number (optional - for debugging)
            if (sequenceNum != expectedSeq) {
                printf("Warning: Sequence mismatch (expected: %d, got: %d)\n",
                       expectedSeq, sequenceNum);
            }
            expectedSeq = (expectedSeq + 1) % 256;

            // Validate data length
            if (4 + dataLength > packetSize) {
                printf("Error: Invalid data packet length\n");
                continue;
            }

            // Write data to file
            fwrite(&packet[4], 1, dataLength, file);
            totalReceived += dataLength;
            packetCount++;

            // Progress indicator
            if (packetCount % 10 == 0 || totalReceived >= expectedFileSize) {
                printf("Progress: %ld/%ld bytes (%.1f%%)\n",
                       totalReceived, expectedFileSize,
                       (totalReceived * 100.0) / expectedFileSize);
            }
        }
        else {
            printf("Warning: Unknown control field: 0x%02X\n", controlField);
        }
    }

    fclose(file);
    printf("File reception complete: %d packets, %ld bytes\n", packetCount, totalReceived);

    if (totalReceived == expectedFileSize) {
        printf("File transfer successful!\n");
        return 0;
    }
    else {
        printf("Warning: File size mismatch!\n");
        return -1;
    }
}

// -------------------- MAIN APPLICATION LAYER FUNCTION --------------------

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    // Setup link layer parameters
    LinkLayer ll;
    strncpy(ll.serialPort, serialPort, sizeof(ll.serialPort) - 1);
    ll.serialPort[sizeof(ll.serialPort) - 1] = '\0';

    if (strcmp(role, "tx") == 0) {
        ll.role = LlTx;
    }
    else if (strcmp(role, "rx") == 0) {
        ll.role = LlRx;
    }
    else {
        printf("Error: Invalid role '%s'. Use 'tx' or 'rx'\n", role);
        return;
    }

    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;

    printf("=== Application Layer ===\n");
    printf("Role: %s\n", role);
    printf("Serial Port: %s\n", serialPort);
    printf("Baud Rate: %d\n", baudRate);
    printf("Retries: %d\n", nTries);
    printf("Timeout: %d seconds\n", timeout);
    printf("=========================\n\n");

    // Open connection
    printf("Opening connection...\n");
    if (llopen(ll) < 0) {
        printf("Error: Failed to establish connection\n");
        return;
    }
    printf("Connection established!\n\n");

    // Perform file transfer
    int result = -1;
    if (ll.role == LlTx) {
        result = transmitFile(&ll, filename);
    }
    else {
        result = receiveFile(&ll, filename);
    }

    // Close connection
    printf("\nClosing connection...\n");
    if (llclose(ll.role) < 0) {
        printf("Warning: Error during connection closure\n");
    }
    else {
        printf("Connection closed successfully\n");
    }

    // Final status
    printf("\n=== Transfer Summary ===\n");
    if (result == 0) {
        printf("Status: SUCCESS ✓\n");
        printf("File: %s\n", filename);
    }
    else {
        printf("Status: FAILED ✗\n");
    }
    printf("========================\n");
}