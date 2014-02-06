#include <stdio.h>

///////////////////////////////////////////////////////////////////////////
//
// Note: The floating point results will very from version to version that 
//       reason probably lays on certain shortcuts that the compiler 
//       optimization took. It may have done doubles rather than floats in
//       some cases. But all the results are in the expected tolerance of
//       the floating point numbers. You can force the compiler to only use
//       floats, but this will be a waste of time.
//
///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////
// PRAGMAS
///////////////////////////////////////////////////////////////////////////
#pragma warning( disable: 4786 ) // identifier was truncated to '255' characters in the debug information

#pragma inline_depth( 255 )
#pragma inline_recursion( on )
#pragma auto_inline( on )

#define inline __forceinline 

///////////////////////////////////////////////////////////////////////////
// VECTOR
///////////////////////////////////////////////////////////////////////////

namespace vector
{
///////////////////////////////////////////////////////////////////////////
// ARGUMENTS
///////////////////////////////////////////////////////////////////////////

    template< class ta_a >
    class vecarg
    {
        const ta_a& Argv;
    public:
        inline vecarg( const ta_a& A ) : Argv( A ) {}
        inline const float Evaluate( const int i ) const 
        { return Argv.Evaluate( i ); }
    };

    template<> 
    class vecarg< const float >
    {
        const ta_a& Argv;
    public:
        inline vecarg( const ta_a& A ) : Argv( A ) {}
        inline const float Evaluate( const int i ) const 
        { return Argv; }
    };

    template<> 
    class vecarg< const int >
    {
        const ta_a& Argv;
    public:
        inline vecarg( const ta_a& A ) : Argv( A ) {}
        inline const float Evaluate( const int i ) const 
        { return (float)Argv; }
    };

///////////////////////////////////////////////////////////////////////////
// EXPRESSIONS
///////////////////////////////////////////////////////////////////////////

    template< class ta_a, class ta_b, class ta_eval >
    class vecexp_2
    {
        const vecarg<ta_a>   Arg1;
        const vecarg<ta_b>   Arg2;

    public:
        inline vecexp_2( const ta_a& A1, const ta_b& A2 ) : Arg1( A1 ), Arg2( A2 ) {}
        inline const float Evaluate ( const int i ) const
        { return ta_eval::Evaluate( i, Arg1, Arg2 ); }
    };

    template< class ta_a, class ta_eval >
    class vecexp_1
    {
        const vecarg<ta_a>   Arg1;

    public:
        inline vecexp_1( const ta_a& A1 ) : Arg1( A1 ) {}

        inline const float Evaluate( const int i ) const
        { return ta_eval::Evaluate( i, Arg1.Evaluate( i ) ); }
    };

///////////////////////////////////////////////////////////////////////////
// BASE CLASS
///////////////////////////////////////////////////////////////////////////

    template< int ta_dimension, class T >
    struct base : public T
    {
        inline       float&  operator[]( const int i )       { return ((float*)this)[i]; } 
        inline const float   Evaluate  ( const int i ) const { return ((float*)this)[i]; }

        //////////////////////////////////////////////////////////////////
        // ASSIGMENT
        //////////////////////////////////////////////////////////////////
        template< class ta >
        struct assigment
        {
            template< int I, class R >
            struct recurse
            {
                enum { COUNTER = I+1 };

                static inline void Assign( base<ta_dimension,T>& V, const ta& A ) 
                {
                    V[I] = A.Evaluate( I );
                    recurse<COUNTER,int>::Assign( V, A );
                }
            };

            template<> struct recurse<ta_dimension,int>
            {
                static inline void Assign( base<ta_dimension,T>& V, const ta& A ){}
            };

            static inline void Assign( base<ta_dimension,T>& V, const ta& A )
            {
                recurse<0,int>::Assign( V, A );
            }
        };

        template< class ta_type > inline
        const base<ta_dimension,T>& operator = ( const ta_type& A )
        {
            assigment<ta_type>::Assign( *this, A );
            return *this;
        }
        
        //////////////////////////////////////////////////////////////////
        // DOT PRODUCT
        //////////////////////////////////////////////////////////////////
        template< class ta, class tb >
        struct dot_prod
        {
            template< int I, class R >
            struct recurse
            {
                enum { COUNTER = I+1 };

                static inline float Dot_Prod( const ta& A, const tb& B ) 
                {
                    return A.Evaluate( I ) * B.Evaluate( I ) +
                        recurse<COUNTER,int>::Dot_Prod( A, B );
                }
            };

            template<> struct recurse<ta_dimension,int>
            {
                static inline float Dot_Prod( const ta& A, const tb& B )
                {
                    return 0;
                }
            };

            static inline float Dot_Prod( const ta& A, const tb& B )
            {
                return recurse<0,int>::Dot_Prod( A, B );
            }
        };

        template< class ta_type > inline
        float Dot( const ta_type& A ) const
        {
            return dot_prod<base<ta_dimension,T>,ta_type>::Dot_Prod( *this, A );
        }
    };

///////////////////////////////////////////////////////////////////////////
// SUM
///////////////////////////////////////////////////////////////////////////

    struct sum
    {
        template< class ta_a, class ta_b > inline static 
        const float Evaluate( const int i, const ta_a& A, const ta_b& B )
        { return A.Evaluate(i) + B.Evaluate(i); }
    };

    template< class ta_c1, class ta_c2 > inline 
    const vecexp_2< const ta_c1, const ta_c2, sum > 
    operator + ( const ta_c1& Pa, const ta_c2& Pb )
    {
        return vecexp_2< const ta_c1, const ta_c2, sum >( Pa, Pb );
    }

///////////////////////////////////////////////////////////////////////////
// DATA
///////////////////////////////////////////////////////////////////////////

    struct desc_xyz
    {
        float X, Y, Z;      
    };

    struct desc_xy
    {
        float X, Y;      
    };

    struct desc_uv
    {
        float U, V;      
    };
};

///////////////////////////////////////////////////////////////////////////
// VECTOR3
///////////////////////////////////////////////////////////////////////////

struct vector3 : public vector::base< 3, vector::desc_xyz >
{
    typedef vector::base< 3, vector::desc_xyz > base;

    inline  vector3( const float x, const float y, const float z )
    { X = z; Y = y; Z = z; }

    template< class ta_type > inline
    vector3& operator = ( const ta_type& A )
    { base::operator = ( A ); return *this; }
};

///////////////////////////////////////////////////////////////////////////
// VECTOR2
///////////////////////////////////////////////////////////////////////////

struct vector2 : public vector::base< 2, vector::desc_xy >
{
    typedef vector::base< 2, vector::desc_xy > base;

    inline  vector2( const float x, const float y )
    { X = x; Y = y; }

    template< class ta_type > inline
    vector2& operator = ( const ta_type& A )
    { base::operator = ( A ); return *this; }
};

///////////////////////////////////////////////////////////////////////////
// VECTOR3D
///////////////////////////////////////////////////////////////////////////

struct vector3d
{
    float X, Y, Z;

    inline vector3d( void ){}
    inline vector3d ( const float x, const float y, const float z )
    { X = x; Y = y; Z = z; }

    inline vector3d operator + ( const vector3d& A ) const
    { return vector3d( X + A.X, Y + A.Y, Z + A.Z ); }                

    inline vector3d operator + ( const float A ) const
    { return vector3d( X + A, Y + A, Z + A ); }                

    inline float Dot( const vector3d& A ) const
    { return A.X*X + A.Y*Y + A.Z*Z; }
};

///////////////////////////////////////////////////////////////////////////
// JIM'S VEC3
///////////////////////////////////////////////////////////////////////////

struct vec3
{
    float X,Y,Z;

    inline float operator [] (const int i) const
    { return ((float*)this)[i]; }

    template< class T > inline
    void Evaluate( const T& e )
    {
        X = e[0];
        Y = e[1];
        Z = e[2];
    }

    template< class T > inline
    vec3( const T& e )
    { Evaluate(e); }

    inline vec3( const float a, const float b, const float c )
    { X = a; Y = b; Z = c; }

    template< class T > inline
    vec3& operator = ( const T& e )
    { Evaluate(e); return *this; }

    template< class T > inline
    float Dot( const T& e )
    { return X * e[0] + Y * e[1] + Z * e[2]; }
};

///////////////////////////////////////////////////////////////////////////
// JIM'S EXPRESSIONS
///////////////////////////////////////////////////////////////////////////

template< class T >
class E
{
    const T Expr;    

public:
    inline E( const T& t ) : Expr( t ) {}

    inline float operator[](const int i) const
    { return Expr[i]; }
};

class E<vec3>
{
    const vec3& V;    
public:
    inline E( const vec3& Vc ) : V( Vc ) {}

    inline float operator[](const int i) const
    { return V[i]; }
};

///////////////////////////////////////////////////////////////////////////
// JIM'S SUM
///////////////////////////////////////////////////////////////////////////

template< class LexprT, class RexprT>
class Vec_Sum
{
    const LexprT L;
    const RexprT R;

public:

    inline Vec_Sum( const LexprT& Linit, const RexprT& Rinit ) : L(Linit), R(Rinit) {}

    inline float operator[]( const int i) const
    { return L[i] + R[i]; }

};

template< class LexprT >
class Vec_Sumf
{
    const LexprT L;
    const float  R;

public:

    inline Vec_Sumf( const LexprT& Linit, const float Rinit ) : L(Linit), R(Rinit) {}

    inline float operator[]( const int I ) const
    { return L[I] + R; }
};

inline const
E< Vec_Sum< vec3, vec3 > >
operator + ( const vec3& a, const vec3& b )
{
    return E< Vec_Sum< vec3, vec3 > >( Vec_Sum< vec3, vec3 >(a,b) );
}

template<class A> inline const
E< Vec_Sum< E<A>, E<vec3> > >
operator + ( const E<A>& a, const vec3& b )
{
    return E< Vec_Sum< E<A>, E<vec3> > >(Vec_Sum< E<A>, E<vec3> >(a,b));
}

template<class A> inline const
E< Vec_Sum< E<vec3>, E<A> > >
operator + ( const vec3& a, const E<A>& b )
{
    return E< Vec_Sum< E<vec3>, E<A> > >(Vec_Sum< E<vec3>, E<A> >(a,b));
}

template< class A > inline const
E< Vec_Sumf< E<A> > >
operator + ( const E<A>& a, const float b )
{
    return E< Vec_Sumf< E<A> > >(Vec_Sumf< E<A> >(a,b));
}

inline const
E< Vec_Sumf< E<vec3> > >
operator + ( const vec3& a, const float b )
{
    return E< Vec_Sumf< E<vec3> > >(Vec_Sumf< E<vec3> >(a,b));
}

template<class A, class B> inline const
E< Vec_Sum< E<A>, E<B> > >
operator + ( const E<A>& a, const E<B>& b )
{
    return E< Vec_Sum< E<A>, E<B> > >(Vec_Sum< E<A>, E<B> >(a,b));
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
// TIMER STUFF
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#pragma comment( lib, "kernel32" )

#include <windows.h>            
#include <mmsystem.h>

struct trial
{
    trial( void ) { T = 0; }
    double T;    
    static double temp;

    double GetTime( void )
    {
        LARGE_INTEGER Clock;
        LARGE_INTEGER ClockFreq;
        QueryPerformanceCounter  ( &Clock ); 
        QueryPerformanceFrequency( &ClockFreq ); 
        return (Clock.QuadPart) / (double)(ClockFreq.QuadPart);
    }

    void Start( void ) 
    { 
        temp = GetTime(); 
    }

    void End( void )
    { 
        temp = GetTime() - temp; 
        T += temp; 
    }
};

double trial::temp = 0;

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
// DO THE TRIALS
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

// Tomas'
vector3     g_vA(1,1,1);
vector3     g_vB(2,2,2);
vector3     g_vC(3,3,3);
vector3     g_vD(4,4,4);

// Typical's
vector3d    g_v3A(1,1,1);
vector3d    g_v3B(2,2,2);
vector3d    g_v3C(3,3,3);
vector3d    g_v3D(4,4,4);

// Jim's
vec3        g_vecA(1,1,1);
vec3        g_vecB(2,2,2);
vec3        g_vecC(3,3,3);
vec3        g_vecD(4,4,4);

void main( void )
{

#if 1 // Change the 1 to 0 to see the assembly of a particular test.

    #define BREAK 
    for( int Test=0; Test<6; Test++ )

#else
    #define Test  2
    #define BREAK _asm int 3
#endif
    {
        int         i;
        trial       T[10];
        const int   Count = 100000000;

        //////////////////////////////////////////////////////////
        T[0].Start();
        for( i=0; i< Count; i++ )
        {
            BREAK;
            BREAK;
            if( Test == 0 ) g_vecD = g_vecA;
            if( Test == 1 ) g_vecD = g_vecD + g_vecA;
            if( Test == 2 ) g_vecD = g_vecD + g_vecA + g_vecB;
            if( Test == 3 ) g_vecD = g_vecD + g_vecA + g_vecB + g_vecC;
            if( Test == 4 ) g_vecD = (g_vecD + g_vecA) + (g_vecB + g_vecC);            
            if( Test == 5 ) g_vecD = g_vecB + g_vecA.Dot( g_vecA + (g_vecB + g_vecC) + g_vecA  );
            BREAK;
            BREAK;
        }
        T[0].End();

        //////////////////////////////////////////////////////////
        T[1].Start();
        for( i=0; i< Count; i++ )
        {
            BREAK;
            BREAK;
            if( Test == 0 ) g_vD = g_vA;
            if( Test == 1 ) g_vD = g_vD + g_vA;
            if( Test == 2 ) g_vD = g_vD + g_vA + g_vB;
            if( Test == 3 ) g_vD = g_vD + g_vA + g_vB + g_vC;
            if( Test == 4 ) g_vD = (g_vD + g_vA) + (g_vB + g_vC);
            if( Test == 5 ) g_vD = g_vB + g_vA.Dot( g_vA + (g_vB + g_vC) + g_vA  );
            BREAK;
            BREAK;
        }
        T[1].End();

        //////////////////////////////////////////////////////////
        T[2].Start();
        for( i=0; i< Count; i++ )
        {
            BREAK;
            BREAK;
            if( Test == 0 ) g_v3D = g_v3A;
            if( Test == 1 ) g_v3D = g_v3D + g_v3A;
            if( Test == 2 ) g_v3D = g_v3D + g_v3A + g_v3B;
            if( Test == 3 ) g_v3D = g_v3D + g_v3A + g_v3B + g_v3C;
            if( Test == 4 ) g_v3D = (g_v3D + g_v3A) + (g_v3B + g_v3C);
            if( Test == 5 ) g_v3D = g_v3B + g_v3A.Dot( g_v3A + (g_v3B + g_v3C) + g_v3A );
            BREAK;
            BREAK;
        }
        T[2].End();

        //////////////////////////////////////////////////////////
        T[3].Start();
        for( i=0; i< Count; i++ )
        {
        }
        T[3].End();

        //////////////////////////////////////////////////////////

        printf( "0) %f %f %f\n", g_vecD.X, g_vecD.Y, g_vecD.Z );
        printf( "1) %f %f %f\n", g_vD.X,   g_vD.Y,   g_vD.Z   );
        printf( "2) %f %f %f\n", g_v3D.X,  g_v3D.Y,  g_v3D.Z  );

        printf( "\nTest%d = %f %f %f [%f]\n\n", Test,
                                                T[0].T, 
                                                T[1].T,
                                                T[2].T,
                                                T[3].T );
    }
}

