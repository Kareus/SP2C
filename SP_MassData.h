#pragma once
#ifndef __SP_MASSDATA__
#define __SP_MASSDATA__

namespace SP2
{
	struct SP_MassData
	{
		double mass;
		double inv_mass;

		double inertia;
		double inv_inertia;

		SP_MassData(double mass = 0, double inertia = 0) : mass(mass), inertia(inertia)
		{
			if (mass == 0) inv_mass = 0;
			else inv_mass = 1 / mass;

			if (inertia == 0) inv_inertia = 0;
			else inv_inertia = 1 / inertia;
		}
	};
}
#endif