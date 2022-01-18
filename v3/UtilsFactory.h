#pragma once

#include "AbsFactory.h"
#include "TransitionUtil.h"


class CLASS_DECLSPEC UtilsFactory : public bl::AbsUtilsFactory
{
public:
	bl::AbsTransUtils* getTransUtils();
};

