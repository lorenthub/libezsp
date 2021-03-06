/**
 * @file aps.cpp
 *
 * @brief Handles encoding/decoding of the APS header
 */

#include "aps.h"
#include "../byte-manip.h"

CAPSFrame::CAPSFrame() : cluster_id(0), dest_ep(0), group_id(0), option(), profile_id(0), sequence(0), src_ep(0)
{
}

CAPSFrame::CAPSFrame(const CAPSFrame& other) :
	cluster_id(other.cluster_id),
	dest_ep(other.dest_ep),
	group_id(other.group_id),
	option(other.option),
	profile_id(other.profile_id),
	sequence(other.sequence),
	src_ep(other.src_ep)
{
}

/**
 * @brief SetDefaultAPS : configure a default aps
 * @param i_profile_id  : profile to use
 * @param i_cluster_id  : cluster concerned
 * @param i_dest_ep     : destination endpoin
 * @param i_grp_id      : group id if necessary
 */
void CAPSFrame::SetDefaultAPS( uint16_t i_profile_id, uint16_t i_cluster_id, uint8_t i_dest_ep, uint16_t i_grp_id )
{
  profile_id = i_profile_id;
  cluster_id = i_cluster_id;
  dest_ep = i_dest_ep;
  group_id = i_grp_id;
  sequence = 0;
  src_ep = 1;

}

std::vector<uint8_t> CAPSFrame::GetEmberAPS(void)
{
  std::vector<uint8_t> lo_aps;
  uint16_t l_option;

  lo_aps.push_back( u16_get_lo_u8(profile_id) );
  lo_aps.push_back( u16_get_hi_u8(profile_id) );

  lo_aps.push_back( u16_get_lo_u8(cluster_id) );
  lo_aps.push_back( u16_get_hi_u8(cluster_id) );

  lo_aps.push_back( src_ep );

  lo_aps.push_back( dest_ep );

  l_option = option.GetEmberApsOption();
  lo_aps.push_back( u16_get_lo_u8(l_option) );
  lo_aps.push_back( u16_get_hi_u8(l_option) );

  lo_aps.push_back( u16_get_lo_u8(group_id) );
  lo_aps.push_back( u16_get_hi_u8(group_id) );

  lo_aps.push_back( sequence );

  return lo_aps;
}

void CAPSFrame::SetEmberAPS(std::vector<uint8_t> i_data )
{
  uint8_t l_idx = 0;

  profile_id = dble_u8_to_u16(i_data.at(l_idx+1U), i_data.at(l_idx));
  l_idx++;
  l_idx++;

  cluster_id = dble_u8_to_u16(i_data.at(l_idx+1U), i_data.at(l_idx));
  l_idx++;
  l_idx++;

  src_ep = i_data.at(l_idx++);

  dest_ep = i_data.at(l_idx++);

  option.SetEmberApsOption( dble_u8_to_u16(i_data.at(l_idx+1U), i_data.at(l_idx)) );
  l_idx++;
  l_idx++;

  group_id = dble_u8_to_u16(i_data.at(l_idx+1U), i_data.at(l_idx));
  l_idx++;
  l_idx++;

  sequence = i_data.at(l_idx++);
}

/**
 * This method is a friend of CAPSFrame class
 * swap() is needed within operator=() to implement to copy and swap paradigm
**/
void swap(CAPSFrame& first, CAPSFrame& second) /* nothrow */
{
  using std::swap;	// Enable ADL

  swap(first.cluster_id, second.cluster_id);
  swap(first.dest_ep, second.dest_ep);
  swap(first.group_id, second.group_id);
  swap(first.option, second.option);
  swap(first.profile_id, second.profile_id);
  swap(first.sequence, second.sequence);
  swap(first.src_ep, second.src_ep);
  /* Once we have swapped the members of the two instances... the two instances have actually been swapped */
}

CAPSFrame& CAPSFrame::operator=(CAPSFrame other)
{
  swap(*this, other);
  return *this;
}

