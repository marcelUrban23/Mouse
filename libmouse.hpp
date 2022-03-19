#ifndef LIBMOUSE_HPP
#define LIBMOUSE_HPP


#include <iostream>
#include <vector>
#include <mutex>
#include <libusb-1.0/libusb.h>

#include <unistd.h>     // usleep()

#define DEBUG_FUNCTION_CALL


class Mouse
{
std::mutex m_mutex;

enum
{
    SUCCESS = 0,
    ERROR = -1,
    CMD_TRANSFER_SIZE = 0x40,

    CMD_GPIF_MODE    = 0xC2, // Set GPIF transfer mode

    MOUSE_PRODUCT_ID = 0xee05,
    MOUSE_VENDOR_ID  = 0x2839,
    MOUSE_ADRESS     = 32,      // I2C Mouse spezifische Geraete Adresse
    ENDPOINT_1_OUT   = 0x01,     // Das ist der Config EP (duplex)
    ENDPOINT_1_IN    = 0x81,
    ENDPOINT_6_IN    = 0x86,

    /* Interpretation High Nibble  */
    I2C_ERROR          = 0xEE, /* Allgemeiner Fehler  */
    I2C_ERROR_NO_MOUSE = 0x10,
    I2C_ERROR_XMIT     = 0x20,
    I2C_ERROR_MISC     = 0x40,

    I2C_RETRIES    = 3,
    I2C_SLEEP_TIME = 50,

    // I2C Kommando-Definitionen  (Commandbyte)
    I2C_ERROR_BUSERR      = 0x01,
    I2C_ERROR_NOACK       = 0x02,
    I2C_ERROR_LOCKUP      = 0x03,
    I2C_ERROR_STOP_TIMEOUT = 0x04,

    CMD_SET_FIFO_FLUSH = 0x9D,
    CMD_GET_MODE      = 0x9E,

    CMD_I2C_WRITE    = 0x83, // Write data to I2C bus
    CMD_I2C_READ     = 0x84, // Read data from I2C bus
    CMD_I2C_SPEED    = 0x85, // Set I2C speed
    CMD_I2C_TRANSFER = 0x86, /*  */

    FILTER_BOUNDARIES_BAND_1 = 221, // Übergabe der Filtergrenzen HF-Band1 (string,11)
    FILTER_BOUNDARIES_BAND_2 = 222, // Übergabe der Filtergrenzen HF-Band1 (string,11)
    FILTER_BOUNDARIES_BAND_3 = 223, // Übergabe der Filtergrenzen HF-Band1 (string,11)
    FILTER_BOUNDARIES_BAND_4 = 224, // Übergabe der Filtergrenzen HF-Band1 (string,11)

    CMD_I2C_SET_FREQUENCY = 15 // AD6636

};

libusb_device_handle *m_mouse_dev;


std::string m_error = {};

void
writeCommand(uint8_t command)
{
    std::vector<uint8_t> data_buffer(CMD_TRANSFER_SIZE); /* nimmt die Bytes auf  */
    data_buffer[0] = command;
    int32_t return_value = 0,
            transfered   = 0;

    m_mutex.lock();
    /* Es folgt der eigentliche Sendevorgang.  */
    return_value = libusb_bulk_transfer(m_mouse_dev,
                                        ENDPOINT_1_OUT,
                                        data_buffer.data(),
                                        1,
                                        &transfered,
                                        100);
    m_mutex.unlock();
    /* Rueckgabewert ungleich NULL war nicht erfolgreich.  */
    if(return_value)
    {
        throw std::runtime_error("FEHLER libmouse::writeCommand() "
                                 + std::string(libusb_error_name(return_value)));
    }
}

/// @brief  I2C adressiertes Kommando an den USB Controller senden und
///         Uebertragungsstatus pruefen.
///         Soll die Funktion USBI2CWriteByte() (nutzt bulk_transfer_out())
///          aus der usb2.dll ersetzen.
/// @param  command               zu sendende Daten
/// @return 0: alles normal, ERROR   bei Fehlern
int32_t
i2cWriteCommandByte(uint8_t command)
{
#ifdef DEBUG_FUNCTION_CALL
    std::cerr << "libmouse::i2cWriteCommandByte()" << std::endl;
#endif

    uint8_t data_buffer[CMD_TRANSFER_SIZE] = {0}, // nimmt die Bytes auf
            return_value = 0,  // Rueckgabewert der Uebrtragung
            length       = 0;  // Zaehler fuer den data_buffer

    int32_t transfered   = 0;

    /* Belegung der zu senden Bytes  */
    data_buffer[length++] = CMD_I2C_WRITE; /* CMD_TRANSFER_SIZE  */
    data_buffer[length++] = (uint8_t)(MOUSE_ADRESS << 1);
    data_buffer[length++] = 1; /* Anzahl der zu senden Bytes  */
    data_buffer[length++] = 0x00; /* reserviert fuer Anzahl der Empfangsbytes */
    data_buffer[length++] = command; /* I2C Kommando an den AVR  */

    /* Daten vor Zweitzugriff schuetzen. - noch umzusetzen  */


    return_value = libusb_claim_interface(m_mouse_dev, 0);
    if(return_value)
    {
        fprintf(stderr, "FEHLER i2cTryToContactmouse(1) "
                        "libusb_claim_interface %d\n",
                        return_value);

        return ERROR;
    }

    m_mutex.lock();

    /* Es folgt der eigentliche Sendevorgang.  */
    return_value = libusb_bulk_transfer(m_mouse_dev,
                                        ENDPOINT_1_OUT,
                                        data_buffer,
                                        length,
                                        &transfered,
                                        100);

    /* Rueckgabewert ungleich NULL war nicht erfolgreich.  */
    if(return_value)
    {
        m_mutex.unlock();
        throw std::runtime_error("FEHLER libmouse::i2cWriteCommandByte() "
                                 + std::string(libusb_error_name(return_value)));
    }

    /* Der Rueckgabewert des vorherigen libusb_bulk_transfer() wurde im  */
    /* Puffer gespeichert.  */
    return_value = libusb_bulk_transfer(m_mouse_dev,
                                        ENDPOINT_1_IN,
                                        data_buffer,
                                        512,
                                        &transfered,  // Anzah
                                        100);         // timeout

    /* Zugriffsschutz der Daten aufheben.  */
    m_mutex.unlock();

    return_value = libusb_release_interface(m_mouse_dev, 0);
    if(return_value)
    {
        fprintf(stderr, "FEHLER i2cTryToContactmouse(4) "
                        "libusb_release_interface %d\n",
                        return_value);

        return ERROR;
    }

    /* Rueckgabewert ungleich NULL ist, war Uebertragung nicht erfolgreich.  */
    if(return_value)
    {
#ifdef DEBUG_FUNCTION_CALL
        fprintf(stderr, "FEHLER i2cWriteCommandByte(3) "
                        "libusb_bulk_transfer: %s\n",
                         libusb_error_name(return_value));
#endif
        return ERROR;
    }

    /* Rueckgabewert im Puffer auf Fehler in der Uebrtragung pruefen.  */
    if(I2C_ERROR ==  data_buffer[2])
    {
        i2cCheckErrorCode(data_buffer[3]);

        return ERROR;
    }


    return SUCCESS;
}


/// @brief I2C adressierte Daten an den USB Controller senden und
///        Uebertragungsstatus pruefen. Soll die Funktion USBI2CWriteBytes()
///       (nutzt bulk_transfer_out()) aus der usb2.dll ersetzen.
/// @param  mouse    geoeffnetes USB Geraet (libusb_open())
/// @param  command   I2C Kommando an den ATMEGA
/// @param  data      zu uebertragende Daten
/// @param  bytes     Anzahl der zu sendenden Bytes
/// @return 0: alles normal, ERROR: bei Fehlern
int32_t
i2cWriteData(uint8_t  command, uint8_t *data, uint8_t  bytes)
{
#ifdef DEBUG_FUNCTION_CALL
    fprintf(stdout, "INFO i2cWriteData()\n");
#endif

    uint8_t data_buffer[CMD_TRANSFER_SIZE] = {0}, /* nimmt die Bytes auf  */
            return_value = 0,  /* Rueckgabewert der Uebrtragung  */
            length       = 0;  /* Zaehler fuer den data_buffer  */

   int32_t  transfered   = 0;


    /* Pruefen, ob Anzahl der zu uebertragenden Bytes zu groß ist.  */
    /* 0x30 == 48 Byte  */
    if(bytes > 0x30)
    {
#ifdef DEBUG_FUNCTION_CALL
        fprintf(stderr, "FEHLER i2cWriteData(1) libusb_bulk_transfer\n");
#endif
        return ERROR;
    }

    /* Belegung der zu senden Bytes  */

    /* CMD_TRANSFER_SIZE  */
    data_buffer[length++] = CMD_I2C_WRITE;
    data_buffer[length++] = (uint8_t)(MOUSE_ADRESS << 1);
    /* Anzahl der zu senden Bytes  */
    data_buffer[length++] = bytes;
    /* reserviert fuer Anzahl der Empfangsbytes  */
    data_buffer[length++] = 0x00;
     /* I2C Kommando an den AVR  */
    data_buffer[length++] = command;

    /* Payload in den Uebrtragungspuffer schreiben  */
    for(int w = 0; w < bytes; w++)
    {
        /* Byte fuer Byte scheiben.  */
        data_buffer[length++] = data[w];
    }

    return_value = libusb_claim_interface(m_mouse_dev, 0);
    if(return_value)
    {
        fprintf(stderr, "FEHLER i2cTryToContactmouse(1) "
                        "libusb_claim_interface %d\n",
                        return_value);

        return ERROR;
    }

    /* Daten vor Zweitzugriff schuetzen. - noch umzusetzen  */
//     interface_lock_mutex();

    /* Es folgt der eigentliche Sendevorgang.  */
    return_value = libusb_bulk_transfer(m_mouse_dev,
                                        ENDPOINT_1_OUT,
                                        data_buffer,
                                        length,
                                        &transfered,
                                        0);

    /* Rueckgabewert ungleich NULL war nicht erfolgreich.  */
    if(return_value)
    {
#ifdef DEBUG_FUNCTION_CALL
        fprintf(stderr, "FEHLER i2cWriteData(2) "
                        "libusb_bulk_transfer: %s\n",
                         libusb_error_name(return_value));
#endif
        /* Zugriffsschutz der Daten aufheben.  */
//         interface_release_mutex();
        return ERROR;
    }

    /* Der Rueckgabewert des vorherigen libusb_bulk_transfer() wurde im  */
    /* Puffer gespeichert.  */
    return_value = libusb_bulk_transfer(m_mouse_dev,
                                        ENDPOINT_1_IN,
                                        data_buffer,
                                        4,
                                        &transfered,
                                        0);

    /* Zugriffsschutz der Daten aufheben.  */
//     interface_release_mutex();

    /* Rueckgabewert ungleich NULL ist, war Uebertragung nicht erfolgreich.  */
    if(return_value)
    {
#ifdef DEBUG_FUNCTION_CALL
        fprintf(stderr, "FEHLER i2cWriteData(3) "
                        "libusb_bulk_transfer: %s\n",
                         libusb_error_name(return_value));
#endif
        return ERROR;
    }

    return_value = libusb_release_interface(m_mouse_dev, 0);
    if(return_value)
    {
        fprintf(stderr, "FEHLER i2cTryToContactmouse(4) "
                        "libusb_release_interface %d\n",
                        return_value);

        return ERROR;
    }

    /* Rueckgabewert im Puffer auf Fehler in der Uebrtragung pruefen.  */
    if(data_buffer[2] == I2C_ERROR)
    {
        i2cCheckErrorCode(data_buffer[3]);
        return ERROR;
    }

    return SUCCESS;
}



/// @brief  Daten an den USB Controller senden und Uebertragungsstatus pruefen.
///         Soll die Funktion bulk_transfer_out() aus der usb2.dll ersetzen.
/// @param  data zu empfangende Daten
/// @param  bytes Anzahl der zu sendenden Bytes
/// @return Anzahl der empfangenen Bytes, ERROR bei Fehlern
int32_t
i2cReadData(uint8_t *data, uint8_t bytes)
{
#ifdef DEBUG_FUNCTION_CALL
    fprintf(stdout, "INFO i2cReadData()\n");
#endif

    uint8_t data_buffer[CMD_TRANSFER_SIZE] = {0}; /* nimmt die Bytes auf  */

    int32_t transfered   = 0,
            w            = 0;

    uint8_t return_value = 0,  /* Rueckgabewert der Uebrtragung  */
            length       = 0;  /* Zaehler fuer den data_buffer  */


    /* pruefen, ob Anzahl der zu uebertragenden Bytes zu groß  */
    if(bytes > 0x30)
    {
#ifdef DEBUG_FUNCTION_CALL
        fprintf(stderr, "FEHLER i2cReadData(1) libusb_bulk_transfer\n");
#endif
        return ERROR;
    }

    /* Belegung der zu senden Bytes  */
    data_buffer[length++] = CMD_I2C_READ; /* CMD_TRANSFER_SIZE  */
    data_buffer[length++] = (uint8_t)(MOUSE_ADRESS << 1);

    /* Null, diese Stelle wird bei i2cWriteData genutzt.  */
    data_buffer[length++] = 0x00;
    data_buffer[length++] = bytes; /* Anzahl der zu lesenden Bytes  */

    /* Daten vor Zweitzugriff schuetzen. - noch umzusetzen  */
//     interface_lock_mutex();

    return_value = libusb_claim_interface(m_mouse_dev, 0);
    if(return_value)
    {
        fprintf(stderr, "FEHLER i2cTryToContactmouse(1) "
                        "libusb_claim_interface %d\n",
                        return_value);

        return ERROR;
    }

    /* Es folgt der eigentliche Sendevorgang.  */
    return_value = libusb_bulk_transfer(m_mouse_dev,
                                        ENDPOINT_1_OUT,
                                        data_buffer,
                                        length,
                                        &transfered,
                                        0);

    /* Rueckgabewert ungleich NULL war nicht erfolgreich.  */
    if(return_value)
    {
#ifdef DEBUG_FUNCTION_CALL
        fprintf(stderr, "FEHLER i2cReadData(2) "
                        "libusb_bulk_transfer: %s\n",
                         libusb_error_name(return_value));
#endif
        /* Zugriffsschutz der Daten aufheben.  */
//         interface_release_mutex();

        return ERROR;
    }
//     else
//     {
//         /*  */
//         fprintf(stderr, "INFO i2cReadData(2) transfered: %d\n", transfered);
//     }


    /* Der Rueckgabewert des vorherigen libusb_bulk_transfer() wurde im  */
    /* Puffer gespeichert.  */
    return_value = libusb_bulk_transfer(m_mouse_dev,
                                        ENDPOINT_1_IN,
                                        data_buffer,
                                        length + bytes,
                                        &transfered,
                                        0);

    /* Zugriffsschutz der Daten aufheben.  */
//     interface_release_mutex();

    /* Wenn Rueckgabewert ungleich NULL, war Uebertragung nicht erfolgreich.  */
    if(return_value)
    {
#ifdef DEBUG_FUNCTION_CALL
        fprintf(stderr, "FEHLER i2cReadData(3) "
                        "libusb_bulk_transfer: %s\n",
                         libusb_error_name(return_value));
#endif
        return ERROR;
    }
//     else
//     {
//         fprintf(stderr, "INFO i2cReadData(3) transfered: %d\n", transfered);
//     }

    return_value = libusb_release_interface(m_mouse_dev, 0);
    if(return_value)
    {
        fprintf(stderr, "FEHLER i2cTryToContactmouse(4) "
                        "libusb_release_interface %d\n",
                        return_value);

        return ERROR;
    }

    /* Rueckgabewert im Puffer auf Fehler in der Uebrtragung pruefen.  */
    if(data_buffer[2] == I2C_ERROR)
    {
        i2cCheckErrorCode(data_buffer[3]);

        return ERROR;
    }
#ifdef DEBUG_FUNCTION_CALL
    /* Ausgabe des Pufferinhalts  */
    else
    {
        for(w = 0; w < transfered; w++)
        {
            fprintf(stdout, "%d ", data_buffer[w]);
        }
        fprintf(stdout, "\n");
    }
#endif
    /* Empfangene Daten in den uebergebenen Puffer kopieren.  */
    for(w = 4; w < transfered; w++)
    {
        data[w - 4] = data_buffer[w];
    }


    return transfered;
}



/// @brief Interpretiert das High und das Low Nibble bezueglich des Fehlercodes.
/// @param i2c_error_code_byte
void
i2cCheckErrorCode(uint8_t i2c_error_code_byte)
{
    /* Interpretiert das High Nibble.  */
    switch(i2c_error_code_byte & 0xF0)
    {
        case I2C_ERROR_NO_MOUSE:
        {
            fprintf(stderr, "FEHLER kein Geraet erkannt\n");
            break;
        }
        case I2C_ERROR_XMIT:
        {
            fprintf(stderr, "FEHLER Uebertragungsfehler\n");
            break;
        }
        case I2C_ERROR_MISC:
        {
            fprintf(stderr, "FEHLER MISC Fehler\n");
            break;
        }
        default:
        {
            fprintf(stderr, "FEHLER unbekannt\n");
            break;
        }
    }

    /* Interpretiert das Low Nibble.  */
    switch(i2c_error_code_byte & 0x0F)
    {
        case I2C_ERROR_BUSERR:
        {
            fprintf(stderr, "FEHLER Bus\n");
            break;
        }
        case I2C_ERROR_NOACK:
        {
            fprintf(stderr, "FEHLER keine Antwort\n");
            break;
        }
        case I2C_ERROR_LOCKUP:
        {
            fprintf(stderr, "FEHLER Locked up\n");
            break;
        }
        case I2C_ERROR_STOP_TIMEOUT:
        {
            fprintf(stderr, "FEHLER STOP Zeit abgelaufen\n");
            break;
        }
        default:
        {
            fprintf(stderr, "FEHLER unbekannt\n");
            break;
        }
    }
}


//int set_gpif_mode(libusb_device_handle *usb_device)
//{
//    uint8_t buffer[CMD_TRANSFER_SIZE],
//            return_value,
//            length = 0,
//            retval = 0;

//    int32_t  transfered   = 0;

//    buffer[length++] = CMD_GPIF_MODE;


//    m_mutex.lock();
//    return_value = libusb_bulk_transfer(usb_device,
//                                        ENDPOINT_1_OUT,
//                                        buffer,
//                                        length,
//                                        &transfered,
//                                        0);

//    retval = bulk_transfer_out ( device_id, buffer, length );
//    if ( retval != length )
//    {
//        interface_release_mutex();
//        return E_FAIL;
//    }

//    if ( check_returncode ( device_id,  CMD_GPIF_MODE, buffer, 1 ) != E_OK )
//    {
//        interface_release_mutex();
//        return E_FAIL;
//    }

//	while ( get_current_mode(device_id) != MODE_GPIF );

//    interface_release_mutex();
//    return E_OK;
//}


public:


bool m_is_open = false;

Mouse(void)
{

}
~Mouse(void)
{
    close();
}



/// @brief Versucht angeschlossene MOUSE zu oeffnen
int
open(void)
{
#ifdef DEBUG_FUNCTION_CALL
    std::cerr << "libmouse::open()" << std::endl;
#endif
    libusb_context *ctx = NULL;

    int32_t return_value = 0;
    /* Pruefen, ob libsub-Sitzung initialisiert wurde.  */
    if((return_value = libusb_init(&ctx)))
    {
#ifdef DEBUG
        throw std::runtime_error("FEHLER mouse::open(): "
                                + std::string(libusb_error_name(return_value)));
#endif
        m_error = ("FEHLER Mouse::open(): "
                + std::string(libusb_error_name(return_value)));

        return -1;
    }


    if( ! (m_mouse_dev = libusb_open_device_with_vid_pid(ctx,
                                                        MOUSE_VENDOR_ID,
                                                        MOUSE_PRODUCT_ID)))
    {
#ifdef DEBUG
        throw std::runtime_error("FEHLER mouse::open() "
                           "MOUSE nicht angeschlossen oder sudo vergessen?\n");
#endif
        m_error = ("FEHLER Mouse::open(): "
                   "MOUSE nicht angeschlossen oder sudo vergessen?\n");

        return -1;
    }

    m_is_open = true;

    return 0;
}

void
close(void)
{
#ifdef DEBUG_FUNCTION_CALL
    std::cerr << "libmouse::close()" << std::endl;
#endif
    if(m_is_open)
    {
        libusb_close(m_mouse_dev);
        m_is_open = false;
    }
}

std::string getError(void){return m_error;}
std::vector<std::vector<uint64_t>> getFilter(void){return i2cReadMouseFilter();}


void
setCenterFrequency(int32_t frequency)
{
#ifdef DEBUG_FUNCTION_CALL
    std::cerr << "libmouse::setCenterFrequency()" << std::endl;
#endif
    frequency = std::min(48000000, std::max(30000, frequency));

    std::cerr << "frequency: " << frequency << std::endl;

    std::vector<unsigned char> freq(4);
    freq.push_back(frequency >> 24);
    freq.push_back(frequency >> 16);
    freq.push_back(frequency >> 8);
    freq.push_back(frequency);

    i2cWriteData(CMD_I2C_SET_FREQUENCY, freq.data(), freq.size());
}

/// @brief  Liest die Anzahl und Bestimmt den Typ der in der MOUSE verfuegbaren
///         Filter um diese spaeter in einer GUI darzustellen.
/// @param  mouse
/// @param  filter_bandwidth
/// @param  filter_middlefrq
/// @return Anzahl der gelesenen Filter, ERROR: bei Fehlern
std::vector<std::vector<uint64_t>>
i2cReadMouseFilter(void)
{
#ifdef DEBUG_FUNCTION_CALL
    std::cerr << "INFO libmouse::i2cReadMouseFilter()" << std::endl;
#endif

    /* Variablendefinition  */
    uint8_t buffer  [30] = {0};
    int32_t filter_count = 0, /* Anzahl der Filter  */
            w            = 0; /* einfache Zaehlvariable  */

    if( ! m_mouse_dev)
    {
        throw std::runtime_error("FEHLER libmouse::i2cReadMouseFilter()"
                                 "no mouse");
    }

    /* Command 100: Filteranzahl im AVR auslesen  */
    if(ERROR == i2cWriteCommandByte(100))
    {
        throw std::runtime_error("FEHLER libmouse::i2cReadMouseFilter()"
                                 "i2cWriteCommandByte");
    }

    usleep(1);

    if(ERROR == i2cReadData(buffer, 1))
    {
        throw std::runtime_error("FEHLER libmouse::i2cReadMouseFilter()"
                                 "i2cReadData");
    }

    /* Anzahl der Filter merken.  */
    filter_count = buffer[0];

    /* Auslesen der einzelnen Filterwerte  */
    std::vector<std::vector<uint64_t>> output;
    for(w = 0; w < filter_count; w++)
    {
        /* Die Nummer des Filters als I2C Kommando uebergeben.  */
        if(ERROR == i2cWriteCommandByte(w + 100 + 1))
        {
            throw std::runtime_error("FEHLER libmouse::i2cReadMouseFilter()"
                                     "i2cWriteCommandByte");
        }

        usleep(1);

        /* Command 9: ...  */
        if(ERROR == i2cReadData(buffer, 9))
        {
            throw std::runtime_error("FEHLER libmouse::i2cReadMouseFilter()"
                                     "i2cReadData");
        }


        std::vector<uint64_t> filter(2);

        filter[1] = (buffer[4] << 24) | (buffer[3] << 16)
                  | (buffer[2] <<  8) | (buffer[1]);

        filter[0] = (buffer[8] << 24) | (buffer[7] << 16)
                  | (buffer[6] <<  8) | (buffer[5]);

        output.push_back(filter);
    }


    return output;
}

void
setGPIFMode(void)
{
#ifdef DEBUG_FUNCTION_CALL
    std::cerr << "libmouse::setGPIFMode()" << std::endl;
#endif
    writeCommand(CMD_GPIF_MODE);
}
void
setFIFOFlush(void)
{
#ifdef DEBUG_FUNCTION_CALL
    std::cerr << "libmouse::setFIFOFlush()" << std::endl;
#endif
    i2cWriteCommandByte(CMD_SET_FIFO_FLUSH);
}


/// @brief Holt Daten ab
/// @param output
/// @return Anzahl gelesener Bytes
int32_t
getStreamData(std::vector<unsigned char> &output)
{
#ifdef DEBUG_FUNCTION_CALL
    std::cerr << "libmouse::streamData()" << std::endl;
#endif
    int32_t transfered = 0;
    unsigned char return_value = 0;
    return_value = libusb_bulk_transfer(m_mouse_dev,
                                        ENDPOINT_6_IN,
                                        output.data(),
                                        output.size(),
                                        &transfered,
                                        100);

    return transfered;
}





};


#endif // LIBMOUSE_HPP
