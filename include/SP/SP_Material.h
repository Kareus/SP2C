#pragma once
#ifndef __SP_MATERIAL__
#define __SP_MATERIAL__

namespace SP2
{
	struct SP_Material
	{
		double density;
		double restitution;
		double friction_static = 0;
		double friction_dynamic = 0;

		SP_Material(double density = 0, double resitituion = 0) : density(density), restitution(resitituion) {}
	};

	namespace Material
	{
		static const SP_Material Rock(0.6, 0.1);
		static const SP_Material Wood(0.3, 0.2);
		static const SP_Material Metal(1.2, 0.05);
		static const SP_Material BouncyBall(0.3, 0.8);
		static const SP_Material SuperBall(0.3, 0.95);
		static const SP_Material Pillow(0.1, 0.2);
		static const SP_Material Static(0, 0.4);
	}

}
#endif