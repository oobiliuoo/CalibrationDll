#include "UtilsFactory.h"

bl::AbsTransUtils* UtilsFactory::getTransUtils()
{
		return new TransitionUtil();
}


