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

class QuaternionFunction: public MatrixFunction
{
    protected :
        Interval a_;
        Interval b_;
        Interval c_;
        Interval d_;
        IntervalVector v_;

    public:
        QuaternionFunction(const IntervalVector& vect): MatrixFunction(4, 4), v_(3, Interval(0., 0.))
        {
            a_    = vect[0];
            b_    = vect[1];
            c_    = vect[2];
            d_    = vect[3];
            /*v_[0] = vect[1];
            v_[1] = vect[2];
            v_[2] = vect[3];*/

            mat_[0][0] = a_;
            mat_[1][1] = a_;
            mat_[2][2] = a_;
            mat_[3][3] = a_;

            mat_[1][0] = b_;
            mat_[2][0] = c_;
            mat_[3][0] = d_;

            mat_[0][1] = -b_;
            mat_[0][2] = -c_;
            mat_[0][3] = -d_;

            mat_[1][2] = -d_;
            mat_[1][3] = c_;
            mat_[2][3] = -b_;

            mat_[2][1] = d_;
            mat_[3][1] = -c_;
            mat_[3][2] = b_;
        }
};

#endif
