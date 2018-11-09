

#include <iostream>

#include "../domain/ezsp-dongle.h"

using namespace std;

class CAppDemo : public CEzspHandler
{
public:
    CAppDemo() { 
        dongle = new CEzspDongle(this);
        // uart
        dongle->open(nullptr); 
    }

    ~CAppDemo() { 
        delete dongle;
    }

    void loop(void) {
        while(1)
            ;
    }

    /**
     * Callback
     */
    virtual void ezspTimerHdl(){;}
    virtual void ezspStackStatusHdl( uint8_t i_status ){;}
    virtual void ezspEnergyScanResultHdl( uint8_t i_channel, int8_t i_max_rssi ){;}
    virtual void ezspNetworkFoundHdl( TNwkInfo i_nwk, uint8_t i_last_hop_lqi, int8_t i_last_hop_rssi ){;}
    virtual void ezspScanCompleteHdl( void ){;}
    virtual void ezspChildJoinHdl(){;}
    virtual void ezspRemoteSetBindingHdl(){;}
    virtual void ezspRemoteDeleteBindingHdl(){;}
    virtual void ezspMessageSentUnicastHdl( EEmberStatus i_status, COutZbMessage *ip_out_msg ){;}
    virtual void ezspMessageSentHdl( std::vector<uint8_t> i_rsp_param ){;}
    virtual void ezspPollCompleteHdl(){;}
    virtual void ezspPollHdl(){;}
    virtual void ezspIncomingSenderEUI64Hdl(){;}
    virtual void ezspIncomingMessageHdl( TInMsgInfo i_msg_info, CZigBeeMsg i_msg ){;}
    virtual void ezspIncomingRouteRecordHdl(){;}
    virtual void ezspIncomingManyToOneRouteRequestHdl(){;}
    virtual void ezspIncomingRouteErrorHdl(){;}
    virtual void ezspIdConflictHdl(){;}
    virtual void ezspMacPassthroughMessageHdl(){;}
    virtual void ezspMacFilterMatchMessageHdl( std::vector<uint8_t> l_msg ){;}
    virtual void ezspRawTransmitCompleteHdl(){;}
    virtual void ezspSwitchNetworkKeyHdl(){;}
    virtual void ezspZigbeeKeyEstablishmentHdl(){;}
    virtual void ezspTrustCenterJoinHdl(){;}
    virtual void ezspGenerateCBKEKeysHdl(){;}
    virtual void ezspCalculateSMACSHdl(){;}
    virtual void ezspDSASignHdl(){;}
    virtual void ezspDSAVerifyHdl(){;}
    virtual void ezspMfglibRxHdl( std::vector<uint8_t> l_msg ){;}
    virtual void ezspIncomingBootloadMessageHdl( uint64_t i_ieee, uint8_t i_last_lqi, int8_t l_last_rssi, std::vector<uint8_t> l_msg ){;}
    virtual void ezspBootloadTransmitCompleteHdl(){;}

private:
    CEzspDongle *dongle;
    
};


int main( void )
{
    CAppDemo *app;

    cout << "Starting ezsp test program !" << endl;

    app = new CAppDemo();

    app->loop();

    delete app;

    return 0;
}