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
	// ������� ��������
	// l - ����� ��������
	// ls - ��� ��������, 1 - ������� ��� 0 - ����� 
	class DashElement
	{
	public:
		double l;
		bool ls;
	};
	// ----------------------------
	// �����������
	// ----------------------------
	Dash();
	Dash(std::vector<double> pattern);
	Dash(std::vector<DashElement> pattern);
	// ----------------------------
	// ����������
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
	// ��������� ��������� ���������������� ��������. 
	// ��� �������� ��������, ����� �������� ��������� � 1.
	// �������� �������/����� ���������������
	// �������������, ������������, ������� �
	// �������.
	// ����� ����������� ����� ��������� ��������.
	// ----------------------------
	void setPattern(std::vector<double> pattern);
	// ----------------------------
	// ��������� �������� �������������� ��������
	// � ������ ���������� ���� ���������
	// ----------------------------
	void setPattern(std::vector<DashElement> pattern);
	// ----------------------------
	// �������� ������� �� ��������
	// ���������� ������ � ����� �����������
	// � ��������� [0:1]
	// ----------------------------
	void samplePattern(double start, double end, std::vector<DashElement>& pattern);
	// ----------------------------
	// double shift - �������� ������ � ��������������� ���� [-1:+1]
	// std::vector<double>& pattern - ���������
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
	// ����
	double x1,// ���������� x ��������� �����
	double y1,// ���������� y ��������� �����
	double x2,// ���������� x �������� �����
	double y2,// ���������� y �������� �����
	std::vector<double>& dash, // ������� 
	double dash_offset, // �������� ���� [-1:1]
	// ���������
	std::vector<double>& xp,// x ���������� ����� ��� ��������� ��������.
	std::vector<double>& yp // y ���������� ����� ��� ��������� ��������.
);

void makeDashedPolyline(
	std::vector<double>& pt_nodes_x,	// (����) ������ x-��������� ������ ����� �������� ���������
	std::vector<double>& pt_nodes_y,	// (����) ������ y-��������� ������ ����� �������� ���������
	std::vector<double> dash, //(����) �������
	double dash_offset, //(����) �������� ���� �������� [-1:+1]
	std::vector<double>& xp,//(�����) ������ x-��������� ��� ��������� ���������
	std::vector<double>& yp,//(�����) ������ y-��������� ��� ��������� ���������
	double & r //(����/�����) �������, ����� ��� ��������� ����������� ��������������� �����.
);



	// ----------------------------
	// �����������
	// ----------------------------
	Dash::Dash()
	{
		// ����� ��������� ��������
		totalLength = 1;
		// �������� ���� [-1:1]
		offset = 0;
	}
	Dash::Dash(std::vector<double> pattern)
	{
		setPattern(pattern);
		// �������� ���� [-1:1]
		offset = 0;
	}
	Dash::Dash(std::vector<DashElement> pattern)
	{
		setPattern(pattern);
		// �������� ���� [-1:1]
		offset = 0;
	}
	// ----------------------------
	// ����������
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
	// ��������� ��������� ���������������� ��������. 
	// ��� �������� ��������, ����� �������� ��������� � 1.
	// �������� �������/����� ���������������
	// �������������, ������������, ������� �
	// �������.
	// ����� ����������� ����� ��������� ��������.
	// ----------------------------
	void Dash::setPattern(std::vector<double> pattern)
	{
		// ������� ��������������� �������
		normalized_pattern.clear();
		// ������� ����� ��������� ��������
		totalLength = 0;
		for (auto e : pattern)
		{
			totalLength += e;
		}
		// ���� ����� �������, �������
		if (this->totalLength < eps)
		{	
			std::cout << "pattern length=0" << std::endl;
			return;
		}
		// �������� �������/������
		bool f = true;
		for (auto& e : pattern)
		{
			normalized_pattern.push_back({ e / totalLength,f });
			f = !f;
		}
	}

	// ----------------------------
	// ��������� �������� �������������� ��������
	// � ������ ���������� ���� ���������
	// ----------------------------
	void Dash::setPattern(std::vector<DashElement> pattern)
	{
		normalized_pattern.clear();
		// ��������� �������� ����� ��������
		totalLength = 0;		
		for (auto e : pattern)
		{
			totalLength += e.l;
		}
		// �������, ���� ����� �������
		if (totalLength < eps)
		{
			std::cout << "pattern length=0" << std::endl;
			return;
		}
		// ��������� �������
		for (auto& e : pattern)
		{
			normalized_pattern.push_back({ e.l / totalLength,e.ls });
		}
	}
	// ----------------------------
	// �������� ������� �� ��������
	// ���������� ������ � ����� �����������
	// � ��������� [0:1]
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
		// ����������� �������� ��������
		// ���������� �������� � ��������� [0:1]
		// 0 - ������ ��������
		// 1 - ����� ��������
		pattern.clear();
		int n = 0;
		double sum = 0;
		for (; n < normalized_pattern.size(); ++n)
		{
			Dash::DashElement e = normalized_pattern[n];
			double elStart = sum;
			double elEnd = sum + e.l;
			// ��������� �������� ������� ����� ������ ������ ��������
			// --|-----|--
			if (elStart <= start && elEnd >= end)
			{
				pattern.push_back({ e.l - (start - elStart) - (elEnd - end),e.ls });
				break;
			}
			// ������� ������� ����� ����� ������� � ������
			// �������������� ���������
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
	// double shift - �������� ������ � ��������������� ���� [-1:+1]
	// std::vector<double>& pattern - ���������
	// ----------------------------
	void Dash::shiftPattern(double offset, std::vector<DashElement>& pattern)
	{
		// ����������
		offset = -offset;
		this->offset = offset;
		// ������� ������ ����������
		pattern.clear();
		// ����� �����
		if (offset < 0)
		{
			// ������� ����� �����
			offset = offset - std::ceil(offset);
		}
		// ����� ������
		else
		{
			// ������� ����� �����
			offset = offset - std::floor(offset);
		}
		// ----------------------------
		// ����� �����
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
		// ��������� �������� �������� 
		// � ���������� �����
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
	// ����
	double x1,// ���������� x ��������� �����
	double y1,// ���������� y ��������� �����
	double x2,// ���������� x �������� �����
	double y2,// ���������� y �������� �����
	std::vector<double>& dash, // ������� 
	double dash_offset, // �������� ���� [-1:1]
	// ���������
	std::vector<double>& xp,// x ���������� ����� ��� ��������� ��������.
	std::vector<double>& yp // y ���������� ����� ��� ��������� ��������.
)
{
	// ��������� �������
	std::vector<Dash::DashElement> shifted_dash;
	// ��� ��������� ������� �������� ����� �������
	// (� ������� �� ���������� ����� ����� ��������� ��������)
	std::vector<Dash::DashElement> sampled_dash;
	// ������ ��� ������ � ����������
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
	std::vector<double>& pt_nodes_x,	// (����) ������ x-��������� ������ ����� �������� ���������
	std::vector<double>& pt_nodes_y,	// (����) ������ y-��������� ������ ����� �������� ���������
	std::vector<double> dash, //(����) �������
	double dash_offset, //(����) �������� ���� �������� [-1:+1]
	std::vector<double>& xp,//(�����) ������ x-��������� ��� ��������� ���������
	std::vector<double>& yp,//(�����) ������ y-��������� ��� ��������� ���������
	double & r //(����/�����) �������, ����� ��� ��������� ����������� ��������������� �����.
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

