#ifndef __lib_dvb_scan_h
#define __lib_dvb_scan_h

#include <dvbsi++/service_description_section.h>
#include <dvbsi++/network_information_section.h>
#include <dvbsi++/bouquet_association_section.h>
#include <dvbsi++/program_association_section.h>
#include <dvbsi++/program_map_section.h>

#include <lib/dvb/idemux.h>
#include <lib/dvb/esection.h>
#include <lib/dvb/db.h>
#include <lib/dvb/atsc.h>

struct service
{
	service(unsigned short pmtPid)
		:pmtPid(pmtPid), serviceType(0xFF), scrambled(false)
	{
	}
	unsigned short pmtPid;
	unsigned char serviceType;
	bool scrambled;
};

class eDVBScan: public sigc::trackable, public iObject
{
	DECLARE_REF(eDVBScan);
		/* chid helper functions: */

		/* heuristically determine if onid/tsid is valid */
	int isValidONIDTSID(int orbital_position, eOriginalNetworkID onid, eTransportStreamID tsid);
		/* build dvb namespace */
	eDVBNamespace buildNamespace(eOriginalNetworkID onid, eTransportStreamID tsid, unsigned long hash);

		/* scan resources */
	eUsePtr<iDVBChannel> m_channel;
	ePtr<iDVBDemux> m_demux;

		/* infrastructure */
	void stateChange(iDVBChannel *);
	ePtr<eConnection> m_stateChanged_connection;

		/* state handling */
	RESULT nextChannel();

	RESULT startFilter();
	enum { readyPAT=1, readySDT=2, readyNIT=4, readyBAT=8,
	       validPAT=16, validSDT=32, validNIT=64, validBAT=128, validVCT=256};

		/* scan state variables */
	int m_channel_state;
	int m_ready, m_ready_all;

	std::map<eDVBChannelID, ePtr<iDVBFrontendParameters> > m_new_channels;
	std::map<eDVBChannelID, int> m_tuner_data; // frequency read from tuner for every new channel

	std::map<eServiceReferenceDVB, ePtr<eDVBService> > m_new_services;
	std::map<eServiceReferenceDVB, ePtr<eDVBService> >::iterator m_last_service;

	std::map<unsigned short, service> m_pmts_to_read;
	std::map<unsigned short, service>::iterator m_pmt_in_progress;
	bool m_pmt_running;
	bool m_abort_current_pmt;

	std::list<ePtr<iDVBFrontendParameters> > m_ch_toScan, m_ch_scanned, m_ch_unavailable, m_ch_blindscan;
	ePtr<iDVBFrontendParameters> m_ch_current, m_ch_blindscan_result;
	eDVBChannelID m_chid_current;
	eTransportStreamID m_pat_tsid;

	ePtr<eTable<ServiceDescriptionSection> > m_SDT;
	ePtr<eTable<NetworkInformationSection> > m_NIT;
	ePtr<eTable<BouquetAssociationSection> > m_BAT;
	ePtr<eTable<ProgramAssociationSection> > m_PAT;
	ePtr<eTable<ProgramMapSection> > m_PMT;
	ePtr<eTable<VirtualChannelTableSection> > m_VCT;

	void SDTready(int err);
	void NITready(int err);
	void BATready(int err);
	void PATready(int err);
	void PMTready(int err);
	void VCTready(int err);

	void addKnownGoodChannel(const eDVBChannelID &chid, iDVBFrontendParameters *feparm);
	void addChannelToScan(iDVBFrontendParameters *feparm);

	int sameChannel(iDVBFrontendParameters *ch1, iDVBFrontendParameters *ch2, bool exact, int offset) const;

	void channelDone();

	sigc::signal<void(int)> m_event;
	RESULT processSDT(eDVBNamespace dvbnamespace, const ServiceDescriptionSection &sdt);
	RESULT processVCT(eDVBNamespace dvbnamespace, const VirtualChannelTableSection &vct, int onid);

	int m_flags;
	int m_networkid;
	bool m_usePAT;
	bool m_scan_debug;
	bool m_updateLCN;

	std::map<eDVBChannelID, uint32_t> m_aus_da_flags;
	std::map<eDVBChannelID, std::string> m_default_authorities;
public:
	eDVBScan(iDVBChannel *channel, bool usePAT=true, bool debug=true );
	~eDVBScan();

	enum {
		scanNetworkSearch = 1, scanSearchBAT = 2,
		scanRemoveServices = 4, scanDontRemoveFeeds = 8,
		scanDontRemoveUnscanned = 16,
		clearToScanOnFirstNIT = 32, scanOnlyFree = 64,
		scanBlindSearch = 128 };

	void start(const eSmartPtrList<iDVBFrontendParameters> &known_transponders, int flags, int networkid = 0);

	enum { evtUpdate, evtNewService, evtFinish, evtFail };
	RESULT connectEvent(const sigc::slot<void(int)> &event, ePtr<eConnection> &connection);
	void insertInto(iDVBChannelList *db, bool backgroundscanresult=false);

	void getStats(int &transponders_done, int &transponders_total, int &services);
	void getLastServiceName(std::string &name);
	void getLastServiceRef(std::string &name);
	RESULT getFrontend(ePtr<iDVBFrontend> &);
	RESULT getCurrentTransponder(ePtr<iDVBFrontendParameters> &);
	eDVBChannelID getCurrentChannelID() { return m_chid_current; }
};

#endif
