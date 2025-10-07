/*------------------------------------------------------------------*/
/// @file		DrawGray.h
/// @brief		Draw grayscale image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <memory>

#include "DrawImage.h"
#include "GrayTable.h"

namespace krm
{

class DrawGray : public DrawImage
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	///	@param	[in]	gray_table		Grayscale table
	/*------------------------------------------------------------------*/
	explicit DrawGray(const std::shared_ptr<GrayTable>& gray_table);

	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	virtual ~DrawGray(void);

	/*------------------------------------------------------------------*/
	/// @brief	set gain coefficient
	/*------------------------------------------------------------------*/
	void setGain(float gain);

private:
	std::shared_ptr<GrayTable> gray_table_;

	/*------------------------------------------------------------------*/
	/// @copydoc	DrawImage::convImage
	/*------------------------------------------------------------------*/
	void convImage(const std::vector<uint16_t>& src_img) override;
};

} // namespace krm
