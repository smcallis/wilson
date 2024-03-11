#pragma once
#include <cmath>
#include <vector>
#include <numeric>
#include <functional> 
#include <algorithm>
#include <iostream>

class Dash
{
public:
	// элемеет паттерна
	// l - длина элемента
	// ls - тип элемента, 1 - отрезок или 0 - зазор 
	class DashElement
	{
	public:
		double l;
		bool ls;
	};
	// ----------------------------
	// Конструктор
	// ----------------------------
	Dash();
	Dash(std::vector<double> pattern);
	Dash(std::vector<DashElement> pattern);
	// ----------------------------
	// деструктор
	// ----------------------------
	~Dash();
	// ----------------------------
	// 
	// ----------------------------
	double getTotalLength();
	// ----------------------------
	// 
	// ----------------------------
	double getOffst();
	// ----------------------------
	// установка исходного нормализованного паттерна. 
	// Для удобства расчетов, длина паттерна приведена к 1.
	// Параметр отрезок/зазор устанавливается
	// автоматически, чередованием, начиная с
	// отрезка.
	// Также вычисляется дляна исходного паттерна.
	// ----------------------------
	void setPattern(std::vector<double> pattern);
	// ----------------------------
	// установка рабочего нормированного паттерна
	// с ручной установкой типа элементов
	// ----------------------------
	void setPattern(std::vector<DashElement> pattern);
	// ----------------------------
	// вырезаем кусочкк из паттерна
	// координаты начала и конца нормированы
	// в интервале [0:1]
	// ----------------------------
	void samplePattern(double start, double end, std::vector<DashElement>& pattern);
	// ----------------------------
	// double shift - величина сдвига в нормализованном виде [-1:+1]
	// std::vector<double>& pattern - результат
	// ----------------------------
	void shiftPattern(double offset, std::vector<DashElement>& pattern);
	// ----------------------------
	// 
	// ----------------------------
	void consolidatePattern(std::vector<DashElement>& pattern);
	private:
	// ----------------------------
	// 
	// ----------------------------
	const double eps = DBL_EPSILON;
	double totalLength;
	double offset;
	std::vector<DashElement> normalized_pattern;
};
// 
double makeDashedLineSegment(
	// вход
	double x1,// координата x начальной точки
	double y1,// координата y начальной точки
	double x2,// координата x конечной точки
	double y2,// координата y конечной точки
	std::vector<double>& dash, // паттерн 
	double dash_offset, // смещение фазы [-1:1]
	// результат
	std::vector<double>& xp,// x координаты точек для отрисовки отрезков.
	std::vector<double>& yp // y координаты точек для отрисовки отрезков.
);

void makeDashedPolyline(
	std::vector<double>& pt_nodes_x,	// (вход) вектор x-координат узлоых точек отрезков полилинии
	std::vector<double>& pt_nodes_y,	// (вход) вектор y-координат узлоых точек отрезков полилинии
	std::vector<double> dash, //(вход) паттерн
	double dash_offset, //(вход) смещение фазы паттерна [-1:+1]
	std::vector<double>& xp,//(выход) вектор x-координат для отрисовки полилинии
	std::vector<double>& yp,//(выход) вектор y-координат для отрисовки полилинии
	double & r //(вход/выход) остаток, нужен для отписовки соединенных последовательно линий.
);



	// ----------------------------
	// Конструктор
	// ----------------------------
	Dash::Dash()
	{
		// длина исходного паттерна
		totalLength = 1;
		// смещение фазы [-1:1]
		offset = 0;
	}
	Dash::Dash(std::vector<double> pattern)
	{
		setPattern(pattern);
		// смещение фазы [-1:1]
		offset = 0;
	}
	Dash::Dash(std::vector<DashElement> pattern)
	{
		setPattern(pattern);
		// смещение фазы [-1:1]
		offset = 0;
	}
	// ----------------------------
	// деструктор
	// ----------------------------
	Dash::~Dash()
	{
		normalized_pattern.clear();
	}
	// ----------------------------
	// 
	// ----------------------------
	double Dash::getTotalLength()
	{
		return totalLength;
	}
	// ----------------------------
	// 
	// ----------------------------
	double Dash::getOffst()
	{
		return offset;
	}
	// ----------------------------
	// установка исходного нормализованного паттерна. 
	// Для удобства расчетов, длина паттерна приведена к 1.
	// Параметр отрезок/зазор устанавливается
	// автоматически, чередованием, начиная с
	// отрезка.
	// Также вычисляется дляна исходного паттерна.
	// ----------------------------
	void Dash::setPattern(std::vector<double> pattern)
	{
		// очимтим нормализованный паттерн
		normalized_pattern.clear();
		// считаем дляну исходного паттерна
		totalLength = 0;
		for (auto e : pattern)
		{
			totalLength += e;
		}
		// если длина нулевая, выходим
		if (this->totalLength < eps)
		{	
			std::cout << "pattern length=0" << std::endl;
			return;
		}
		// параметр отрезок/пробел
		bool f = true;
		for (auto& e : pattern)
		{
			normalized_pattern.push_back({ e / totalLength,f });
			f = !f;
		}
	}

	// ----------------------------
	// установка рабочего нормированного паттерна
	// с ручной установкой типа элементов
	// ----------------------------
	void Dash::setPattern(std::vector<DashElement> pattern)
	{
		normalized_pattern.clear();
		// вычисляем исходную дляну паттерна
		totalLength = 0;		
		for (auto e : pattern)
		{
			totalLength += e.l;
		}
		// выходим, если длина нулевая
		if (totalLength < eps)
		{
			std::cout << "pattern length=0" << std::endl;
			return;
		}
		// нормируем паттерн
		for (auto& e : pattern)
		{
			normalized_pattern.push_back({ e.l / totalLength,e.ls });
		}
	}
	// ----------------------------
	// вырезаем кусочкк из паттерна
	// координаты начала и конца нормированы
	// в интервале [0:1]
	// ----------------------------
	void Dash::samplePattern(double start, double end, std::vector<Dash::DashElement>& pattern)
	{
		if (start < 0)
		{
			std::cout << "sampler: start < 0 " << std::endl;
			start = 0;
		}
		if (start > 1)
		{
			std::cout << "sampler: start > 1 " << std::endl;
			start = 1;
		}
		if (end < 0)
		{
			std::cout << "sampler: end < 0 " << std::endl;
			end = 0;
		}
		if (end > 1)
		{
			std::cout << "sampler: end > 0 " << std::endl;
			start = 1;
		}
		if (start > end)
		{
			std::cout << "sampler: start > end " << std::endl;
			std::swap(start, end);
		}
		// вытаскиваем фрагмент паттерна
		// координаты задаются в интервале [0:1]
		// 0 - начало паттерна
		// 1 - конец паттерна
		pattern.clear();
		int n = 0;
		double sum = 0;
		for (; n < normalized_pattern.size(); ++n)
		{
			Dash::DashElement e = normalized_pattern[n];
			double elStart = sum;
			double elEnd = sum + e.l;
			// требуемый интервал целиком лежит внутрм одного сегмента
			// --|-----|--
			if (elStart <= start && elEnd >= end)
			{
				pattern.push_back({ e.l - (start - elStart) - (elEnd - end),e.ls });
				break;
			}
			// элемент целиком лежит между началом и концом
			// запрашиваемого интервала
			// | ------   |
			else if (elStart >= start && elEnd < end)
			{
				pattern.push_back({ e.l, e.ls });
			}
			// --|-----   |
			else if (elStart < start && elEnd < end)
			{
				pattern.push_back({ elEnd - start, e.ls });
			}
			// |   -----|--
			else if (elStart > start && elEnd > end)
			{
				pattern.push_back({ end - elStart, e.ls });
				break;
			}
			sum += e.l;
		}
	}
	// ----------------------------
	// double shift - величина сдвига в нормализованном виде [-1:+1]
	// std::vector<double>& pattern - результат
	// ----------------------------
	void Dash::shiftPattern(double offset, std::vector<DashElement>& pattern)
	{
		// перевернкм
		offset = -offset;
		this->offset = offset;
		// очимтим вектор результата
		pattern.clear();
		// сдвиг влево
		if (offset < 0)
		{
			// убираем целую часть
			offset = offset - std::ceil(offset);
		}
		// сдвиг вправо
		else
		{
			// убираем целую часть
			offset = offset - std::floor(offset);
		}
		// ----------------------------
		// сдвиг влево
		// ----------------------------
		if (std::fabs(offset) > eps)
		{
			int n = 0;
			int split = -1;
			double sum = offset > 0 ? offset : 1 + offset;
			for (DashElement e : normalized_pattern)
			{
				sum -= e.l;
				if (sum < 0 && split == -1)
				{
					pattern.push_back({ e.l + sum , e.ls });
					pattern.push_back({ -sum , e.ls });
					++n;
					split = n;
					continue;
				}
				pattern.push_back(e);
				++n;
			}
			std::vector<DashElement> part1;
			std::vector<DashElement> part2;
			part1.assign(pattern.begin(), pattern.begin() + split);
			part2.assign(pattern.begin() + split, pattern.end());
			pattern.clear();
			pattern.insert(pattern.end(), part2.begin(), part2.end());
			pattern.insert(pattern.end(), part1.begin(), part1.end());
			part1.clear();
			part2.clear();
		}
		else
		{
			pattern.assign(normalized_pattern.begin(), normalized_pattern.end());
		}
		consolidatePattern(pattern);
	}
	// ----------------------------
	// 
	// ----------------------------
	void Dash::consolidatePattern(std::vector<DashElement>& pattern)
	{
		// объединим соседние елементы 
		// с одинаковым типом
		for (int i = 0; i < pattern.size() - 1; ++i)
		{
			if (pattern[i].ls == pattern[i + 1].ls)
			{
				pattern[i].l += pattern[i + 1].l;
				pattern.erase(pattern.begin() + i);
				--i;
			}
		}
	}

// 
double makeDashedLineSegment(
	// вход
	double x1,// координата x начальной точки
	double y1,// координата y начальной точки
	double x2,// координата x конечной точки
	double y2,// координата y конечной точки
	std::vector<double>& dash, // паттерн 
	double dash_offset, // смещение фазы [-1:1]
	// результат
	std::vector<double>& xp,// x координаты точек для отрисовки отрезков.
	std::vector<double>& yp // y координаты точек для отрисовки отрезков.
)
{
	// сдвинутый паттерн
	std::vector<Dash::DashElement> shifted_dash;
	// для отрисовки дробной конечтой части отрезка
	// (в которую не помещается целая длина исходного паттерна)
	std::vector<Dash::DashElement> sampled_dash;
	// движок для работы с паттернами
	Dash lineDash;
	lineDash.setPattern(dash);
	lineDash.shiftPattern(dash_offset, shifted_dash);
	double originalPatternLength = lineDash.getTotalLength();
	double liineSegmentLength = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	if (liineSegmentLength < DBL_EPSILON)
	{
		return 0;
	}
	int times = std::floor(liineSegmentLength / originalPatternLength);
	double reminder = (liineSegmentLength - times * originalPatternLength) / originalPatternLength;
	double ortx = (x2 - x1) / liineSegmentLength;
	double orty = (y2 - y1) / liineSegmentLength;
	double sum = 0;
	for (int j = 0; j < times; ++j)
	{
		for (int i = 0; i < shifted_dash.size(); ++i)
		{
			if (shifted_dash[i].ls == 1)
			{
				double xstart = x1 + sum * ortx * originalPatternLength;
				double ystart = y1 + sum * orty * originalPatternLength;
				xp.push_back(xstart);
				xp.push_back(xstart + shifted_dash[i].l * originalPatternLength * ortx);
				
				yp.push_back(ystart);
				yp.push_back(ystart + shifted_dash[i].l * originalPatternLength * orty);
			}
			sum += shifted_dash[i].l;
		}
	}
	lineDash.setPattern(shifted_dash);

	double start = 0;
	double end = reminder;

	lineDash.samplePattern(start, end, sampled_dash);

	for (int i = 0; i < sampled_dash.size(); ++i)
	{
		if (sampled_dash[i].ls == 1)
		{
			double xstart = x1 + sum * ortx * originalPatternLength;
			double ystart = y1 + sum * orty * originalPatternLength;
			xp.push_back(xstart);
			xp.push_back(xstart + sampled_dash[i].l * originalPatternLength * ortx);

			yp.push_back(ystart);
			yp.push_back(ystart + sampled_dash[i].l * originalPatternLength * orty);
		}
		sum += sampled_dash[i].l;
	}
	return reminder;
}

void makeDashedPolyline(
	std::vector<double>& pt_nodes_x,	// (вход) вектор x-координат узлоых точек отрезков полилинии
	std::vector<double>& pt_nodes_y,	// (вход) вектор y-координат узлоых точек отрезков полилинии
	std::vector<double> dash, //(вход) паттерн
	double dash_offset, //(вход) смещение фазы паттерна [-1:+1]
	std::vector<double>& xp,//(выход) вектор x-координат для отрисовки полилинии
	std::vector<double>& yp,//(выход) вектор y-координат для отрисовки полилинии
	double & r //(вход/выход) остаток, нужен для отписовки соединенных последовательно линий.
)
{
	double x1 = 0;
	double y1 = 0;
	double x2 = 0;
	double y2 = 0;
	for (int i = 0; i < pt_nodes_x.size() - 1; i += 1)
	{
		x1 = pt_nodes_x[i];
		y1 = pt_nodes_y[i];
		x2 = pt_nodes_x[i + 1];
		y2 = pt_nodes_y[i + 1];
		r += makeDashedLineSegment(
			x1, y1,
			x2, y2,
			dash, dash_offset - r,
			xp, yp);
	}
}

