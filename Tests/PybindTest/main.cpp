#include <pybind11/pybind11.h>
#include <eigen3/Eigen/Dense>
#include <pybind11/pytypes.h>
#include <iostream>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j = 0) {
    return i + j;
}


namespace py = pybind11;


void EigenTest(py::buffer InBuf)
{
    // typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> Strides;
    using Strides = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
    using Scalar = double;

    /* Request a buffer descriptor from Python */
    py::buffer_info info = InBuf.request();

    /* Some basic validation checks ... */
    if (info.format != py::format_descriptor<Scalar>::format())
        throw std::runtime_error("Incompatible format: expected a double array!");

    if (info.ndim != 2)
        throw std::runtime_error("Incompatible buffer dimension!");

    auto strides = Strides(
        info.strides[1] / (py::ssize_t)sizeof(Scalar),
        info.strides[0] / (py::ssize_t)sizeof(Scalar));

    auto map = Eigen::Map<Eigen::MatrixXd, 0, Strides>(
        static_cast<Scalar *>(info.ptr), info.shape[0], info.shape[1], strides);

    std::cout << map << std::endl;
}

PYBIND11_MODULE(pybind11_test, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: cmake_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    m.def("add", &add, R"pbdoc(
        Add two numbers

        Some other explanation about the add function.
    )pbdoc");

    m.def("subtract", [](int i, int j) { return i - j; }, R"pbdoc(
        Subtract two numbers

        Some other explanation about the subtract function.
    )pbdoc");

    m.def("eigen_test", &EigenTest);

    /* Bind MatrixXd (or some other Eigen type) to Python */
    typedef Eigen::MatrixXd Matrix;

    typedef Matrix::Scalar Scalar;
    constexpr bool rowMajor = Matrix::Flags & Eigen::RowMajorBit;

    py::class_<Matrix>(m, "Matrix", py::buffer_protocol())
        .def(py::init([](py::buffer b) {
            typedef Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic> Strides;

            /* Request a buffer descriptor from Python */
            py::buffer_info info = b.request();

            /* Some basic validation checks ... */
            if (info.format != py::format_descriptor<Scalar>::format())
                throw std::runtime_error("Incompatible format: expected a double array!");

            if (info.ndim != 2)
                throw std::runtime_error("Incompatible buffer dimension!");

            auto strides = Strides(
                info.strides[rowMajor ? 0 : 1] / (py::ssize_t)sizeof(Scalar),
                info.strides[rowMajor ? 1 : 0] / (py::ssize_t)sizeof(Scalar));

            auto map = Eigen::Map<Matrix, 0, Strides>(
                static_cast<Scalar *>(info.ptr), info.shape[0], info.shape[1], strides);

            return Matrix(map);
        }));


    

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}