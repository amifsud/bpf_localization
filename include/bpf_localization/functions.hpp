#ifndef GEOMETRY
#define GEOMETRY

class SpecializedFunction
{
    protected:
        Function* func_;
        Array<const ExprNode> nodes_;

    protected:
        virtual void computeNodes(Variable* x, unsigned int i) = 0;

        void define_func(Variable* x, unsigned int i)
        {
            computeNodes(x, i);
	        const ExprVector& vec = ExprVector::new_(nodes_, false);
            func_ = new Function(*x, vec);
        }

    public:
        SpecializedFunction()
        {
        }

        const ExprApply& operator()(Variable* state, unsigned int i)
        {
            define_func(state, i);
            return func_->operator()(*state);
        }
};

class SubVariable: public Variable
{
    protected:
        unsigned int index_;
        unsigned int sub_size_;

    public:
        SubVariable(unsigned int size, unsigned int i, unsigned int sub_size)
            : Variable(size)
        {
            index_ = i;
            sub_size_ = sub_size;
        }

        unsigned int index()
        {
            return index_;
        }

        unsigned int sub_size()
        {
            return sub_size_;
        }
};

class MatrixFunction: public SpecializedFunction 
{
    protected:
        IntervalMatrix mat_;

    public:
        MatrixFunction(const IntervalMatrix& mat): SpecializedFunction(), mat_(mat)
        {
            ROS_ASSERT(mat.nb_rows() > 1 && "A matrix contains at least two rows");
            ROS_ASSERT(mat.nb_cols() > 1 && "A matrix contains at least two cols");
            mat_ = mat;
        }

        MatrixFunction(unsigned int rows, unsigned int cols)
            : mat_(rows, cols, Interval(0., 0.))
        {
        }

        void computeNodes(Variable* x, unsigned int i)
        {
            Array<const ExprAdd> sums;
            for(unsigned int row = 0; row < mat_.nb_rows(); row++)
            {
                sums.clear();
                for(unsigned int col = 1; col < mat_.nb_cols(); col++)
                {
                    if(col == 1)
                    {
                        sums.add(ExprAdd::new_(mat_[row][0]*x->operator[](i),
                                            mat_[row][1]*x->operator[](i+1)));
                    }
                    else
                    {
                        sums.add(ExprAdd::new_(sums[sums.size()-1],
                                            mat_[row][col]*x->operator[](i+col)));
                    }
                }
                nodes_.add(sums[sums.size()-1]);
            }
        }

        const ExprApply& operator*(SubVariable* x)
        {
            return this->operator()(x, x->index()); 
        }

        IntervalMatrix getIntervalMatrix()
        {
            return mat_;
        }

        static MatrixFunction identity(unsigned int size)
        {
            IntervalMatrix m(size, size, Interval(0., 0.));

            for(auto i = 0; i < size; i++) m[i][i] = Interval(1., 1.);

            return MatrixFunction(m); 
        }
};

class TranslationFunction: public SpecializedFunction
{
    protected:
        IntervalVector trans_;

    public:
        TranslationFunction(const IntervalVector& vect): SpecializedFunction(), trans_(vect)
        {
        }

        void computeNodes(Variable* x, unsigned int i)
        {
            for(unsigned int u = 0; u < trans_.size(); ++u)
                nodes_.add(x->operator[](i+u)+trans_[u]);
        }

        const ExprApply& operator+(SubVariable* x)
        {
            return this->operator()(x, x->index()); 
        }
};

const ExprApply& operator+(SubVariable* x, TranslationFunction& t)
{
    return t.operator()(x, x->index()); 
}


class CrossProductFunction: public MatrixFunction
{
    protected:
        IntervalVector v_;

    public:
        CrossProductFunction(const IntervalVector& vect)
            : MatrixFunction(3, 3), v_(3, Interval(0., 0.))
        {
            mat_[0][1] = -v_[2];
            mat_[0][2] =  v_[1];
            mat_[1][2] = -v_[0];
            mat_[1][0] =  v_[2];
            mat_[2][0] = -v_[1];
            mat_[2][1] =  v_[0];
        }
};

class QuaternionFunction: public MatrixFunction
{
    protected :
        Interval       a_;
        IntervalVector v_;

    public:
        QuaternionFunction(const IntervalVector& vect)
            : MatrixFunction(4, 4), v_(3, Interval(0., 0.))
        {
            a_    = vect[0];
            v_[0] = vect[1];
            v_[1] = vect[2];
            v_[2] = vect[3];

            mat_ = a_*MatrixFunction::identity(4).getIntervalMatrix();

            mat_.put(0, 1, -v_, true);
            mat_.put(1, 0,  v_, false);

            IntervalMatrix m = a_*MatrixFunction::identity(3).getIntervalMatrix()
                                + CrossProductFunction(v_).getIntervalMatrix();
            mat_.put(1, 1, m);
        }

        QuaternionFunction(const Interval& a, const IntervalVector& vect)
            : MatrixFunction(4, 4), v_(3, Interval(0., 0.))
        {
            a_    = a;
            v_[0] = vect[0];
            v_[1] = vect[1];
            v_[2] = vect[2];

            mat_ = a_*MatrixFunction::identity(4).getIntervalMatrix();

            mat_.put(0, 1, -v_, true);
            mat_.put(1, 0,  v_, false);

            IntervalMatrix m = a_*MatrixFunction::identity(3).getIntervalMatrix()
                                + CrossProductFunction(v_).getIntervalMatrix();
            mat_.put(1, 1, m);
        }

        QuaternionFunction(const IntervalMatrix& mat)
            : MatrixFunction(mat), v_(3, Interval(0., 0.))
        {
            a_    = mat_[0][0];
            v_[0] = mat_[1][0];
            v_[1] = mat_[2][0];
            v_[2] = mat_[3][0];
        }

        const ExprApply& operator*(SubVariable* x)
        {
            return this->operator()(x, x->index());
        }

        QuaternionFunction rightProduct()
        {
            IntervalMatrix mat = this->getIntervalMatrix();
            IntervalMatrix cross = a_*MatrixFunction::identity(3).getIntervalMatrix()
                                    + CrossProductFunction(v_).getIntervalMatrix();
            mat.submatrix(1, 1, 3, 3) -= 2*cross;

            return QuaternionFunction(mat);
        }

        QuaternionFunction inverse()
        {
            return QuaternionFunction(a_, -v_);
        }
};

const ExprApply& operator*(SubVariable* x, QuaternionFunction& q)
{
    QuaternionFunction q_right_product = q.rightProduct();
    return q_right_product.operator()(x, x->index()); 
}

#endif
