/*------------------------------------------------------------------*/
/// @file		DrawThread.cpp
/// @brief		Draw image Conversion Thread class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include "DrawThread.h"
#include "DrawPcd.h"

namespace krm
{

DrawThread::DrawThread(DrawImages& draws) :
	EvtThread(), draws_(draws), snap_taking_(false)
{
}

DrawThread::~DrawThread(void)
{
	for (auto& draw : draws_) {
		draw.reset();
	}
}

Result DrawThread::recvChgProp(const CameraProperty& property)
{
	que_th_ = 1U;
	for (uint8_t i = 0; i < IMG_KINDS; i++) {
		draws_[i]->setAveCycle(property.mode_info.fps);
	}
	return SUCCESS;
}


Result DrawThread::recvChgFmt(const ImageFormats& formats)
{
	for (uint8_t i = 0; i < IMG_KINDS; i++) {
		draws_[i]->setFormat(formats[i]);
	}
	std::dynamic_pointer_cast<DrawPcd>(draws_[IMG_PCD])->setFormat(formats[IMG_DEPTH]);
	return SUCCESS;
}

Result DrawThread::recvImage(FrameData& frame)
{
	bool ret = false;
	uint8_t i;
	Frame* frame_data = frame.getFrame();
	if (snap_taking_) {
		for (i = 0; i < IMG_KINDS; i++) {
			ret |= draws_[i]->setSnapShot(frame_data->images[i]);
		}
		ret |= std::dynamic_pointer_cast<DrawPcd>(draws_[IMG_PCD])->setSnapShot(frame_data->pcd);
	} else {
		for (i = 0; i < IMG_KINDS; i++) {
			draws_[i]->setImage(frame_data->images[i]);
		}
		std::dynamic_pointer_cast<DrawPcd>(draws_[IMG_PCD])->setImage(frame_data->pcd);
	}
	return (ret ? REACH_EOF : SUCCESS);
}

void DrawThread::recvReset(void)
{
	for (auto& draw : draws_) {
		draw->clear();
	}
}

Result DrawThread::recvUserEvent(uint8_t event, void* param)
{
	Result ret = SUCCESS;

	switch (event) {
	case EV_SNAP_TAKE:
		if (param == nullptr) {
			return ERR_INVALID_PTR;
		}
		for (auto& draw : draws_) {
			draw->startSnapshot(*(static_cast<uint8_t*>(param)));
		}
		snap_taking_ = true;
		break;
	case EV_SNAP_EXIT:
		for (auto& draw : draws_) {
			draw->endSnapshot();
		}
		snap_taking_ = false;
		break;
	default:
		ret = ERR_BAD_ARG;
		break;
	}
	return ret;
}

} // namespace krm
