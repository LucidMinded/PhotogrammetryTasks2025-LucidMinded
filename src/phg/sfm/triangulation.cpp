#include "triangulation.h"

#include <Eigen/SVD>

#include "defines.h"

// По положениям камер и ключевых точкам определяем точку в трехмерном пространстве
// Задача эквивалентна поиску точки пересечения двух (или более) лучей
// Используем DLT метод, составляем систему уравнений. Система похожа на систему для гомографии, там пары уравнений получались из выражений вида x (cross) Hx = 0, а здесь будет x (cross) PX = 0
// (см. Hartley & Zisserman p.312)
cv::Vec4d phg::triangulatePoint(const cv::Matx34d *Ps, const cv::Vec3d *ms, int count) {
    Eigen::MatrixXd A(2 * count, 4);
    for (int i = 0; i < count; ++i) {
        const cv::Matx34d &P = Ps[i];
        const cv::Vec3d &m = ms[i];
        auto firstRow = m[0] * P.row(2) - P.row(0);
        auto secondRow = m[1] * P.row(2) - P.row(1);

        A(2 * i + 0, 0) = firstRow(0);
        A(2 * i + 0, 1) = firstRow(1);
        A(2 * i + 0, 2) = firstRow(2);
        A(2 * i + 0, 3) = firstRow(3);

        A(2 * i + 1, 0) = secondRow(0);
        A(2 * i + 1, 1) = secondRow(1);
        A(2 * i + 1, 2) = secondRow(2);
        A(2 * i + 1, 3) = secondRow(3);
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::VectorXd X = svd.matrixV().col(3);
    return cv::Vec4d(X[0], X[1], X[2], X[3]);
}
