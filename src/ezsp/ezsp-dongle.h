/**
 * @file ezsp-dongle.h
 *
 * @brief Handles EZSP communication with a dongle over a serial port
**/

#pragma once

#include <string>
#include <iostream>
#include <vector>
#include <queue>

#include "ezsp-protocol/ezsp-enum.h"
#include "spi/IUartDriver.h"
#include "ash.h"
#include "ezsp-dongle-observer.h"
#include "spi/TimerBuilder.h"

extern "C" {	/* Avoid compiler warning on member initialization for structs (in -Weffc++ mode) */
    typedef struct sMsg
    {
        EEzspCmd i_cmd;	/*!< The EZSP command to send */
        std::vector<uint8_t> payload;	/*!< The payload for the EZSP command (as a byte buffer) */
    }SMsg;
}

#ifdef USE_RARITAN
/**** Start of the official API; no includes below this point! ***************/
#include <pp/official_api_start.h>
#endif // USE_RARITAN

class CEzspDongle : public IAsyncDataInputObserver, public CAshCallback
{
public:
    /**
     * @brief Constructor
     *
     * @param[in] i_timer_factory A timer builder
     * @param[in] p_observer An optional observer that will be notified when dongle state changes and when a EZSP message is received
     *
     * @note Observers can also be registered later on using method registerObserver()
     */
    CEzspDongle( TimerBuilder &i_timer_factory, CEzspDongleObserver* ip_observer = nullptr );

    CEzspDongle() = delete; // Construction without arguments is not allowed
    CEzspDongle(const CEzspDongle&) = delete; /* No copy construction allowed (pointer data members) */

    /**
     * @brief Destructor
     */
    ~CEzspDongle();

    CEzspDongle& operator=(CEzspDongle) = delete; /* No assignment allowed (pointer data members) */

    /**
     * @brief Open an EZSP connection (via a serial port) to a dongle
     *
     * @param ipUart A pointer to the UART driver to use for serial I/O
     *
     * @return true if we could successfully reset the EZSP communication on the serial port
     */
    bool open(IUartDriver *ipUart);

    /**
     * @brief Send an EZSP command to the dongle
     *
     * @param i_cmd The EZSP command to send
     * @param i_cmd_payload The payload
     */
    void sendCommand(EEzspCmd i_cmd, std::vector<uint8_t> i_cmd_payload = std::vector<uint8_t>() );

    /**
     * @brief Callback invoked on UART received bytes
     *
     * @param dataIn The pointer to the incoming bytes buffer
     * @param dataLen The size of the data to read inside dataIn
     */
    void handleInputData(const unsigned char* dataIn, const size_t dataLen);

    /**
     * @brief Callback invoked on ASH info
     *
     * @param info An information pushed by the ASH layer
     */
    void ashCbInfo( EAshInfo info );

    /**
     * Managing Observer of this class
     */
    bool registerObserver(CEzspDongleObserver* observer);
    bool unregisterObserver(CEzspDongleObserver* observer);

private:
    TimerBuilder &timer_factory;        /*!< A timer builder used to handle timeouts */
    IUartDriver *pUart; /*!< The UART used for EZSP communication */
    CAsh *ash;  /*!< The ASH instance used to carry EZSP messages */
    GenericAsyncDataInputObservable uartIncomingDataHandler;    /*!< The incoming data handler that we will observe to read bytes from the serial port */
    std::queue<SMsg> sendingMsgQueue;   /*!< The EZSP messages queued to be sent to the dongle */
    bool wait_rsp;      /*!< Are we currently waiting for an EZSP response to an EZSP command we have sent? */

    /**
     * @brief Send the next message in our EZSP message queue (sendingMsgQueue)
     */
    void sendNextMsg( void );

    /**
     * Notify Observer of this class
     */
    std::set<CEzspDongleObserver*> observers;
    void notifyObserversOfDongleState( EDongleState i_state );
    void notifyObserversOfEzspRxMessage( EEzspCmd i_cmd, std::vector<uint8_t> i_message );
};

#ifdef USE_RARITAN
#include <pp/official_api_end.h>
#endif // USE_RARITAN
