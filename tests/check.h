 /*
  * Check: a unit test framework for C
  * Copyright (C) 2001, 2002 Arien Malec
  *
  * This library is free software; you can redistribute it and/or
  * modify it under the terms of the GNU Lesser General Public
  * License as published by the Free Software Foundation; either
  * version 2.1 of the License, or (at your option) any later version.
  *
  * This library is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  * Lesser General Public License for more details.
  *
  * You should have received a copy of the GNU Lesser General Public
  * License along with this library; if not, write to the
  * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
  * MA 02110-1301, USA.
  */
 
 #ifndef CHECK_H
 #define CHECK_H
 
 #include <stddef.h>
 #include <string.h>
 #include <math.h>
 #include <float.h>
 
// #include <check_stdint.h>
 
 /*
    Macros and functions starting with _ (underscore) are internal and
    may change without notice. You have been warned!.
 */
 
 
 #ifdef __cplusplus
 #define CK_CPPSTART extern "C" {
 #define CK_CPPEND }
 CK_CPPSTART
 #endif
 
 #if defined(__GNUC__) && defined(__GNUC_MINOR__)
 #define GCC_VERSION_AT_LEAST(major, minor) \
 ((__GNUC__ > (major)) || \
  (__GNUC__ == (major) && __GNUC_MINOR__ >= (minor)))
 #else
 #define GCC_VERSION_AT_LEAST(major, minor) 0
 #endif
 
 #if GCC_VERSION_AT_LEAST(2,95)
 #define CK_ATTRIBUTE_UNUSED __attribute__ ((unused))
 #else
 #define CK_ATTRIBUTE_UNUSED
 #endif /* GCC 2.95 */
 
 #if GCC_VERSION_AT_LEAST(2,5)
 #define CK_ATTRIBUTE_NORETURN __attribute__ ((noreturn))
 #else
 #define CK_ATTRIBUTE_NORETURN
 #endif /* GCC 2.5 */
 
 #include <sys/types.h>
 
 /*
  * Used to create the linker script for hiding lib-local symbols. Shall
  * be put directly in front of the exported symbol.
  */
 #define CK_EXPORT
 
 /*
  * Used for MSVC to create the export attribute
  * CK_DLL_EXP is defined during the compilation of the library
  * on the command line.
  */
 #ifndef CK_DLL_EXP
 #define CK_DLL_EXP
 #endif
 
 /* check version numbers */
 
 #define CHECK_MAJOR_VERSION (0)
 #define CHECK_MINOR_VERSION (11)
 #define CHECK_MICRO_VERSION (0)
 
 CK_DLL_EXP extern int CK_EXPORT check_major_version;
 CK_DLL_EXP extern int CK_EXPORT check_minor_version;
 CK_DLL_EXP extern int CK_EXPORT check_micro_version;
 
 #ifndef NULL
 #define NULL ((void*)0)
 #endif
 
 typedef struct TCase TCase;
 
 typedef void (*TFun) (int);
 
 typedef void (*SFun) (void);
 
 typedef struct Suite Suite;
 
 CK_DLL_EXP Suite *CK_EXPORT suite_create(const char *name);
 
 CK_DLL_EXP int CK_EXPORT suite_tcase(Suite * s, const char *tcname);
 
 CK_DLL_EXP void CK_EXPORT suite_add_tcase(Suite * s, TCase * tc);
 
 CK_DLL_EXP TCase *CK_EXPORT tcase_create(const char *name);
 
 CK_DLL_EXP void CK_EXPORT tcase_set_tags(TCase * tc,
                                          const char *tags);
 #define tcase_add_test(tc,tf) tcase_add_test_raise_signal(tc,tf,0)
 
 #define tcase_add_test_raise_signal(tc,tf,signal) \
    _tcase_add_test((tc),(tf),"" # tf "",(signal), 0, 0, 1)
 
 #define tcase_add_exit_test(tc, tf, expected_exit_value) \
   _tcase_add_test((tc),(tf),"" # tf "",0,(expected_exit_value),0,1)
 
 #define tcase_add_loop_test(tc,tf,s,e) \
   _tcase_add_test((tc),(tf),"" # tf "",0,0,(s),(e))
 
 #define tcase_add_loop_test_raise_signal(tc,tf,signal,s,e) \
   _tcase_add_test((tc),(tf),"" # tf "",(signal),0,(s),(e))
 
 #define tcase_add_loop_exit_test(tc,tf,expected_exit_value,s,e) \
   _tcase_add_test((tc),(tf),"" # tf "",0,(expected_exit_value),(s),(e))
 
 /* Add a test function to a test case
   (function version -- use this when the macro won't work
 */
 CK_DLL_EXP void CK_EXPORT _tcase_add_test(TCase * tc, TFun tf,
                                           const char *fname, int _signal,
                                           int allowed_exit_value, int start,
                                           int end);
 
 CK_DLL_EXP void CK_EXPORT tcase_add_unchecked_fixture(TCase * tc, SFun setup,
                                                       SFun teardown);
 
 CK_DLL_EXP void CK_EXPORT tcase_add_checked_fixture(TCase * tc, SFun setup,
                                                     SFun teardown);
 
 CK_DLL_EXP void CK_EXPORT tcase_set_timeout(TCase * tc, double timeout);
 
 /* Internal function to mark the start of a test function */
 CK_DLL_EXP void CK_EXPORT tcase_fn_start(const char *fname, const char *file,
                                          int line);
 
 CK_DLL_EXP const char* CK_EXPORT tcase_name();
 
 #define START_TEST(__testname)\
 static void __testname (int _i CK_ATTRIBUTE_UNUSED)\
 {\
   tcase_fn_start (""# __testname, __FILE__, __LINE__);
 
 #define END_TEST }
 
 /*
  * Fail the test case unless expr is false
  *
  * This call is deprecated.
  */
 #define fail_unless ck_assert_msg
 
 /*
  * Fail the test case if expr is false
  *
  * This call is deprecated.
  *
  * NOTE: The space before the comma sign before ## is essential to be compatible
  * with gcc 2.95.3 and earlier.
  * FIXME: these macros may conflict with C89 if expr is
  * FIXME:   strcmp (str1, str2) due to excessive string length.
  */
 #define fail_if(expr, ...)\
   (expr) ? \
      _ck_assert_failed(__FILE__, __LINE__, "Failure '"#expr"' occurred" , ## __VA_ARGS__, NULL) \
      : _mark_point(__FILE__, __LINE__)
 
 /*
  * Fail the test
  *
  * This call is deprecated.
  */
 #define fail ck_abort_msg
 
 /*
  * This is called whenever an assertion fails.
  * Note that it only has the noreturn modifier when
  * using fork. If fork is unavailable, the function
  * calls longjmp() when a test assertion fails. Marking
  * the function as noreturn causes gcc to make assumptions
  * which are not valid, as longjmp() is like a return.
  */
 #if 1
 CK_DLL_EXP void CK_EXPORT _ck_assert_failed(const char *file, int line,
                                             const char *expr,
                                             ...) CK_ATTRIBUTE_NORETURN;
 #else
 CK_DLL_EXP void CK_EXPORT _ck_assert_failed(const char *file, int line,
                                             const char *expr, ...);
 #endif
 
 #define ck_assert(expr) ck_assert_msg(expr, NULL)
 
 /* The space before the comma sign before ## is essential to be compatible
    with gcc 2.95.3 and earlier.
 */
 #define ck_assert_msg(expr, ...) \
   (expr) ? \
      _mark_point(__FILE__, __LINE__) : \
      _ck_assert_failed(__FILE__, __LINE__, "Assertion '"#expr"' failed" , ## __VA_ARGS__, NULL)
 
 #define ck_abort() ck_abort_msg(NULL)
 
 #define ck_abort_msg(...) _ck_assert_failed(__FILE__, __LINE__, "Failed" , ## __VA_ARGS__, NULL)
 
 /* Signed and unsigned integer comparison macros with improved output compared to ck_assert(). */
 /* OP may be any comparison operator. */
 #define _ck_assert_int(X, OP, Y) do { \
   intmax_t _ck_x = (X); \
   intmax_t _ck_y = (Y); \
   ck_assert_msg(_ck_x OP _ck_y, "Assertion '%s' failed: %s == %jd, %s == %jd", #X" "#OP" "#Y, #X, _ck_x, #Y, _ck_y); \
 } while (0)
 
 #define ck_assert_int_eq(X, Y) _ck_assert_int(X, ==, Y)
 
 #define ck_assert_int_ne(X, Y) _ck_assert_int(X, !=, Y)
 
 #define ck_assert_int_lt(X, Y) _ck_assert_int(X, <, Y)
 
 #define ck_assert_int_le(X, Y) _ck_assert_int(X, <=, Y)
 
 #define ck_assert_int_gt(X, Y) _ck_assert_int(X, >, Y)
 
 #define ck_assert_int_ge(X, Y) _ck_assert_int(X, >=, Y)
 
 #define _ck_assert_uint(X, OP, Y) do { \
   uintmax_t _ck_x = (X); \
   uintmax_t _ck_y = (Y); \
   ck_assert_msg(_ck_x OP _ck_y, "Assertion '%s' failed: %s == %ju, %s == %ju", #X" "#OP" "#Y, #X, _ck_x, #Y, _ck_y); \
 } while (0)
 
 #define ck_assert_uint_eq(X, Y) _ck_assert_uint(X, ==, Y)
 
 #define ck_assert_uint_ne(X, Y) _ck_assert_uint(X, !=, Y)
 
 #define ck_assert_uint_lt(X, Y) _ck_assert_uint(X, <, Y)
 
 #define ck_assert_uint_le(X, Y) _ck_assert_uint(X, <=, Y)
 
 #define ck_assert_uint_gt(X, Y) _ck_assert_uint(X, >, Y)
 
 #define ck_assert_uint_ge(X, Y) _ck_assert_uint(X, >=, Y)
 
 /* Number of digits after the decimal point to output via printf */
 #ifndef CK_FLOATING_DIG
 # define CK_FLOATING_DIG 6
 #endif /* CK_FLOATING_DIG */
 
 /* Floating point number comparison macros with improved output
  * compared to ck_assert(). */
 /* OP may be any comparison operator, TP is type, TM is type modifier. */
 #define _ck_assert_floating(X, OP, Y, TP, TM) do { \
   TP _ck_x = (X); \
   TP _ck_y = (Y); \
   ck_assert_msg(_ck_x OP _ck_y, \
   "Assertion '%s' failed: %s == %.*"TM"g, %s == %.*"TM"g", \
   #X" "#OP" "#Y, \
   #X, (int)CK_FLOATING_DIG, _ck_x, \
   #Y, (int)CK_FLOATING_DIG, _ck_y); \
 } while (0)
 
 /* Check floating point number is finise. */
 /* TP is type, TM is type modifier. */
 #define _ck_assert_floating_finite(X, TP, TM) \
 do { \
   TP _ck_x = (X); \
   ck_assert_msg(isfinite(_ck_x), \
     "Assertion '%s' failed: %s == %.*"TM"g", \
     #X" is finite", \
     #X, (int)CK_FLOATING_DIG, _ck_x); \
 } while (0)
 
 /* Check floating point number is infinise. */
 /* TP is type, TM is type modifier. */
 #define _ck_assert_floating_infinite(X, TP, TM) \
 do { \
   TP _ck_x = (X); \
   ck_assert_msg(isinf(_ck_x), \
     "Assertion '%s' failed: %s == %.*"TM"g", \
     #X" is infinite", \
     #X, (int)CK_FLOATING_DIG, _ck_x); \
 } while (0)
 
 /* Check floating point number is "Not a Number". */
 /* TP is type, TM is type modifier. */
 #define _ck_assert_floating_nan(X, TP, TM) \
 do { \
   TP _ck_x = (X); \
   ck_assert_msg(isnan(_ck_x), \
     "Assertion '%s' failed: %s == %.*"TM"g", \
     #X" is NaN", \
     #X, (int)CK_FLOATING_DIG, _ck_x); \
 } while (0)
 
 /* Check floating point number is not "Not a Number". */
 /* TP is type, TM is type modifier. */
 #define _ck_assert_floating_nonnan(X, TP, TM) \
 do { \
   TP _ck_x = (X); \
   ck_assert_msg(!isnan(_ck_x), \
     "Assertion '%s' failed: %s == %.*"TM"g", \
     #X" is not NaN", \
     #X, (int)CK_FLOATING_DIG, _ck_x); \
 } while (0)
 
 /* Floating point tolerance comparison macros with improved output
  * compared to ck_assert(). */
 /* OP, D can have values: >, -1; <, 1. */
 #define _ck_assert_floating_op_tol(X, OP, Y, T, D, TP, TM) do { \
   TP _ck_x = (X); \
   TP _ck_y = (Y); \
   TP _ck_t = (T); \
   ck_assert_msg((_ck_x - _ck_y) OP _ck_t * (D), \
   "Assertion '%s' failed: %s == %.*"TM"g, %s == %.*"TM"g, %s == %.*"TM"g", \
   #X" "#OP"= "#Y", error < "#T, \
   #X, (int)CK_FLOATING_DIG, _ck_x, \
   #Y, (int)CK_FLOATING_DIG, _ck_y, \
   #T, (int)CK_FLOATING_DIG, _ck_t); \
 } while (0)
 
 /* Floating point tolerance comparison macros with improved output
  * compared to ck_assert(). */
 /* OP can have values: <; >=. */
 #define _ck_assert_floating_absdiff_op_tol(X, Y, OP, T, TP, TM) \
 do { \
   TP _ck_x = (X); \
   TP _ck_y = (Y); \
   TP _ck_t = (T); \
   ck_assert_msg(fabsl(_ck_y - _ck_x) OP _ck_t, \
     "Assertion '%s' failed: %s == %.*"TM"g, %s == %.*"TM"g, %s == %.*"TM"g", \
     "fabsl("#Y" - "#X") "#OP" "#T, \
     #X, (int)CK_FLOATING_DIG, _ck_x, \
     #Y, (int)CK_FLOATING_DIG, _ck_y, \
     #T, (int)CK_FLOATING_DIG, _ck_t); \
 } while (0)
 
 #define ck_assert_float_eq(X, Y) _ck_assert_floating(X, ==, Y, float, "")
 
 #define ck_assert_float_ne(X, Y) _ck_assert_floating(X, !=, Y, float, "")
 
 #define ck_assert_float_lt(X, Y) _ck_assert_floating(X, <, Y, float, "")
 
 #define ck_assert_float_le(X, Y) _ck_assert_floating(X, <=, Y, float, "")
 
 #define ck_assert_float_gt(X, Y) _ck_assert_floating(X, >, Y, float, "")
 
 #define ck_assert_float_ge(X, Y) _ck_assert_floating(X, >=, Y, float, "")
 
 #define ck_assert_float_eq_tol(X, Y, T)  _ck_assert_floating_absdiff_op_tol(X, Y, <, T, float, "")
 
 #define ck_assert_float_ne_tol(X, Y, T) _ck_assert_floating_absdiff_op_tol(X, Y, >=, T, float, "")
 
 #define ck_assert_float_ge_tol(X, Y, T) _ck_assert_floating_op_tol(X, >, Y, T, -1, float, "")
 
 #define ck_assert_float_le_tol(X, Y, T) _ck_assert_floating_op_tol(X, <, Y, T, 1, float, "")
 
 #define ck_assert_float_finite(X) _ck_assert_floating_finite(X, float, "")
 
 #define ck_assert_float_infinite(X) _ck_assert_floating_infinite(X, float, "")
 
 #define ck_assert_float_nan(X) _ck_assert_floating_nan(X, float, "")
 
 #define ck_assert_float_nonnan(X) _ck_assert_floating_nonnan(X, float, "l")
 
 #define ck_assert_double_eq(X, Y) _ck_assert_floating(X, ==, Y, double, "l")
 
 #define ck_assert_double_ne(X, Y) _ck_assert_floating(X, !=, Y, double, "l")
 
 #define ck_assert_double_lt(X, Y) _ck_assert_floating(X, <, Y, double, "l")
 
 #define ck_assert_double_le(X, Y) _ck_assert_floating(X, <=, Y, double, "l")
 
 #define ck_assert_double_gt(X, Y) _ck_assert_floating(X, >, Y, double, "l")
 
 #define ck_assert_double_ge(X, Y) _ck_assert_floating(X, >=, Y, double, "l")
 
 #define ck_assert_double_eq_tol(X, Y, T)  _ck_assert_floating_absdiff_op_tol(X, Y, <, T, double, "l")
 
 #define ck_assert_double_ne_tol(X, Y, T) _ck_assert_floating_absdiff_op_tol(X, Y, >=, T, double, "l")
 
 #define ck_assert_double_ge_tol(X, Y, T) _ck_assert_floating_op_tol(X, >, Y, T, -1, double, "l")
 
 #define ck_assert_double_le_tol(X, Y, T) _ck_assert_floating_op_tol(X, <, Y, T, 1, double, "l")
 
 #define ck_assert_double_finite(X) _ck_assert_floating_finite(X, double, "l")
 
 #define ck_assert_double_infinite(X) _ck_assert_floating_infinite(X, double, "l")
 
 #define ck_assert_double_nan(X) _ck_assert_floating_nan(X, double, "l")
 
 #define ck_assert_double_nonnan(X) _ck_assert_floating_nonnan(X, double, "l")
 
 #define ck_assert_ldouble_eq(X, Y) _ck_assert_floating(X, ==, Y, long double, "L")
 
 #define ck_assert_ldouble_ne(X, Y) _ck_assert_floating(X, !=, Y, long double, "L")
 
 #define ck_assert_ldouble_lt(X, Y) _ck_assert_floating(X, <, Y, long double, "L")
 
 #define ck_assert_ldouble_le(X, Y) _ck_assert_floating(X, <=, Y, long double, "L")
 
 #define ck_assert_ldouble_gt(X, Y) _ck_assert_floating(X, >, Y, long double, "L")
 
 #define ck_assert_ldouble_ge(X, Y) _ck_assert_floating(X, >=, Y, long double, "L")
 
 #define ck_assert_ldouble_eq_tol(X, Y, T)  _ck_assert_floating_absdiff_op_tol(X, Y, <, T, long double, "L")
 
 #define ck_assert_ldouble_ne_tol(X, Y, T) _ck_assert_floating_absdiff_op_tol(X, Y, >=, T, long double, "L")
 
 #define ck_assert_ldouble_ge_tol(X, Y, T) _ck_assert_floating_op_tol(X, >, Y, T, -1, long double, "L")
 
 #define ck_assert_ldouble_le_tol(X, Y, T) _ck_assert_floating_op_tol(X, <, Y, T, 1, long double, "L")
 
 #define ck_assert_ldouble_finite(X) _ck_assert_floating_finite(X, long double, "L")
 
 #define ck_assert_ldouble_infinite(X) _ck_assert_floating_infinite(X, long double, "L")
 
 #define ck_assert_ldouble_nan(X) _ck_assert_floating_nan(X, long double, "L")
 
 #define ck_assert_ldouble_nonnan(X) _ck_assert_floating_nonnan(X, long double, "L")
 
 /* String comparison macros with improved output compared to ck_assert() */
 /* OP might be any operator that can be used in '0 OP strcmp(X,Y)' comparison. */
 /* String pointer could be compared againts NULL with == (NULLEQ = 1) and != (NULLNE = 1) operators. */
 /* The x and y parameter swap in strcmp() is needed to handle >, >=, <, <= operators. */
 /* If the x or y parameter is NULL its value will be printed without quotes. */
 #define _ck_assert_str(X, OP, Y, NULLEQ, NULLNE) do { \
   const char* _ck_x = (X); \
   const char* _ck_y = (Y); \
   const char* _ck_x_s; \
   const char* _ck_y_s; \
   const char* _ck_x_q; \
   const char* _ck_y_q; \
   if (_ck_x != NULL) { \
     _ck_x_q = "\""; \
     _ck_x_s = _ck_x; \
   } else { \
     _ck_x_q = ""; \
     _ck_x_s = "(null)"; \
   } \
   if (_ck_y != NULL) { \
     _ck_y_q = "\""; \
     _ck_y_s = _ck_y; \
   } else { \
     _ck_y_q = ""; \
     _ck_y_s = "(null)"; \
   } \
   ck_assert_msg( \
     (NULLEQ && (_ck_x == NULL) && (_ck_y == NULL)) || \
     (NULLNE && ((_ck_x == NULL) || (_ck_y == NULL)) && (_ck_x != _ck_y)) || \
     ((_ck_x != NULL) && (_ck_y != NULL) && (0 OP strcmp(_ck_y, _ck_x))), \
     "Assertion '%s' failed: %s == %s%s%s, %s == %s%s%s", \
     #X" "#OP" "#Y, \
     #X, _ck_x_q, _ck_x_s, _ck_x_q, \
     #Y, _ck_y_q, _ck_y_s, _ck_y_q); \
 } while (0)
 
 #define ck_assert_str_eq(X, Y) _ck_assert_str(X, ==, Y, 0, 0)
 
 #define ck_assert_str_ne(X, Y) _ck_assert_str(X, !=, Y, 0, 0)
 
 #define ck_assert_str_lt(X, Y) _ck_assert_str(X, <, Y, 0, 0)
 
 #define ck_assert_str_le(X, Y) _ck_assert_str(X, <=, Y, 0, 0)
 
 #define ck_assert_str_gt(X, Y) _ck_assert_str(X, >, Y, 0, 0)
 
 #define ck_assert_str_ge(X, Y) _ck_assert_str(X, >=, Y, 0, 0)
 
 #define ck_assert_pstr_eq(X, Y) _ck_assert_str(X, ==, Y, 1, 0)
 
 #define ck_assert_pstr_ne(X, Y) _ck_assert_str(X, !=, Y, 0, 1)
 
 /* Memory location comparison macros with improved output compared to ck_assert() */
 /* OP might be any operator that can be used in '0 OP memcmp(X,Y,L)' comparison */
 /* The x and y parameter swap in memcmp() is needed to handle >, >=, <, <= operators */
 /* Output is limited to CK_MAX_ASSERT_MEM_PRINT_SIZE bytes */
 #ifndef CK_MAX_ASSERT_MEM_PRINT_SIZE
 #define CK_MAX_ASSERT_MEM_PRINT_SIZE 64
 #endif
 
 /* Memory location comparison macros with improved output compared to ck_assert() */
 /* OP might be any operator that can be used in '0 OP memcmp(X,Y,L)' comparison */
 /* The x and y parameter swap in memcmp() is needed to handle >, >=, <, <= operators */
 /* Output is limited to CK_MAX_ASSERT_MEM_PRINT_SIZE bytes */
 #ifndef CK_MAX_ASSERT_MEM_PRINT_SIZE
 #define CK_MAX_ASSERT_MEM_PRINT_SIZE 64
 #endif
 
 #define _ck_assert_mem(X, OP, Y, L) do { \
   const uint8_t* _ck_x = (const uint8_t*)(X); \
   const uint8_t* _ck_y = (const uint8_t*)(Y); \
   size_t _ck_l = (L); \
   char _ck_x_str[CK_MAX_ASSERT_MEM_PRINT_SIZE * 2 + 1]; \
   char _ck_y_str[CK_MAX_ASSERT_MEM_PRINT_SIZE * 2 + 1]; \
   static const char _ck_hexdigits[] = "0123456789abcdef"; \
   size_t _ck_i; \
   size_t _ck_maxl = (_ck_l > CK_MAX_ASSERT_MEM_PRINT_SIZE) ? CK_MAX_ASSERT_MEM_PRINT_SIZE : _ck_l; \
   for (_ck_i = 0; _ck_i < _ck_maxl; _ck_i++) { \
     _ck_x_str[_ck_i * 2  ]   = _ck_hexdigits[(_ck_x[_ck_i] >> 4) & 0xF]; \
     _ck_y_str[_ck_i * 2  ]   = _ck_hexdigits[(_ck_y[_ck_i] >> 4) & 0xF]; \
     _ck_x_str[_ck_i * 2 + 1] = _ck_hexdigits[_ck_x[_ck_i] & 0xF]; \
     _ck_y_str[_ck_i * 2 + 1] = _ck_hexdigits[_ck_y[_ck_i] & 0xF]; \
   } \
   _ck_x_str[_ck_i * 2] = 0; \
   _ck_y_str[_ck_i * 2] = 0; \
   if (_ck_maxl != _ck_l) { \
     _ck_x_str[_ck_i * 2 - 2] = '.'; \
     _ck_y_str[_ck_i * 2 - 2] = '.'; \
     _ck_x_str[_ck_i * 2 - 1] = '.'; \
     _ck_y_str[_ck_i * 2 - 1] = '.'; \
   } \
   ck_assert_msg(0 OP memcmp(_ck_y, _ck_x, _ck_l), \
     "Assertion '%s' failed: %s == \"%s\", %s == \"%s\"", #X" "#OP" "#Y, #X, _ck_x_str, #Y, _ck_y_str); \
 } while (0)
 
 #define ck_assert_mem_eq(X, Y, L) _ck_assert_mem(X, ==, Y, L)
 
 #define ck_assert_mem_ne(X, Y, L) _ck_assert_mem(X, !=, Y, L)
 
 #define ck_assert_mem_lt(X, Y, L) _ck_assert_mem(X, <, Y, L)
 
 #define ck_assert_mem_le(X, Y, L) _ck_assert_mem(X, <=, Y, L)
 
 #define ck_assert_mem_gt(X, Y, L) _ck_assert_mem(X, >, Y, L)
 
 #define ck_assert_mem_ge(X, Y, L) _ck_assert_mem(X, >=, Y, L)
 
 /* Pointer comparison macros with improved output compared to ck_assert(). */
 /* OP may only be == or !=  */
 #define _ck_assert_ptr(X, OP, Y) do { \
   const void* _ck_x = (X); \
   const void* _ck_y = (Y); \
   ck_assert_msg(_ck_x OP _ck_y, "Assertion '%s' failed: %s == %#x, %s == %#x", #X" "#OP" "#Y, #X, _ck_x, #Y, _ck_y); \
 } while (0)
 
 /* Pointer against NULL comparison macros with improved output
  * compared to ck_assert(). */
 /* OP may only be == or !=  */
 #define _ck_assert_ptr_null(X, OP) do { \
   const void* _ck_x = (X); \
   ck_assert_msg(_ck_x OP NULL, \
   "Assertion '%s' failed: %s == %#x", \
   #X" "#OP" NULL", \
   #X, _ck_x); \
 } while (0)
 
 #define ck_assert_ptr_eq(X, Y) _ck_assert_ptr(X, ==, Y)
 
 #define ck_assert_ptr_ne(X, Y) _ck_assert_ptr(X, !=, Y)
 
 #define ck_assert_ptr_null(X) _ck_assert_ptr_null(X, ==)
 
 #define ck_assert_ptr_nonnull(X) _ck_assert_ptr_null(X, !=)
 
 #define mark_point() _mark_point(__FILE__,__LINE__)
 
 /* Non macro version of #mark_point */
 CK_DLL_EXP void CK_EXPORT _mark_point(const char *file, int line);
 
 enum test_result
 {
     CK_TEST_RESULT_INVALID,     
     CK_PASS,                    
     CK_FAILURE,                 
     CK_ERROR                    
 };
 
 enum print_output
 {
     CK_SILENT,                  
     CK_MINIMAL,                 
     CK_NORMAL,                  
     CK_VERBOSE,                 
     CK_ENV,                     
 #if 0
     CK_SUBUNIT,                 
 #endif
     CK_LAST                     
 };
 
 typedef struct SRunner SRunner;
 
 typedef struct TestResult TestResult;
 
 enum ck_result_ctx
 {
     CK_CTX_INVALID,             
     CK_CTX_SETUP,               
     CK_CTX_TEST,                
     CK_CTX_TEARDOWN             
 };
 
 CK_DLL_EXP int CK_EXPORT tr_rtype(TestResult * tr);
 
 CK_DLL_EXP enum ck_result_ctx CK_EXPORT tr_ctx(TestResult * tr);
 
 CK_DLL_EXP const char *CK_EXPORT tr_msg(TestResult * tr);
 
 CK_DLL_EXP int CK_EXPORT tr_lno(TestResult * tr);
 
 CK_DLL_EXP const char *CK_EXPORT tr_lfile(TestResult * tr);
 
 CK_DLL_EXP const char *CK_EXPORT tr_tcname(TestResult * tr);
 
 CK_DLL_EXP SRunner *CK_EXPORT srunner_create(Suite * s);
 
 CK_DLL_EXP void CK_EXPORT srunner_add_suite(SRunner * sr, Suite * s);
 
 CK_DLL_EXP void CK_EXPORT srunner_free(SRunner * sr);
 
 CK_DLL_EXP void CK_EXPORT srunner_run_all(SRunner * sr,
                                           enum print_output print_mode);
 
 CK_DLL_EXP void CK_EXPORT srunner_run(SRunner * sr, const char *sname,
                                       const char *tcname,
                                       enum print_output print_mode);
 
 
 CK_DLL_EXP void CK_EXPORT srunner_run_tagged(SRunner * sr, const char *sname,
                                              const char *tcname,
                                              const char *include_tags,
                                              const char *exclude_tags,
                                              enum print_output print_mode);
 
 CK_DLL_EXP int CK_EXPORT srunner_ntests_failed(SRunner * sr);
 
 CK_DLL_EXP int CK_EXPORT srunner_ntests_run(SRunner * sr);
 
 CK_DLL_EXP TestResult **CK_EXPORT srunner_failures(SRunner * sr);
 
 CK_DLL_EXP TestResult **CK_EXPORT srunner_results(SRunner * sr);
 
 CK_DLL_EXP void CK_EXPORT srunner_print(SRunner * sr,
                                         enum print_output print_mode);
 
 CK_DLL_EXP void CK_EXPORT srunner_set_log(SRunner * sr, const char *fname);
 
 CK_DLL_EXP int CK_EXPORT srunner_has_log(SRunner * sr);
 
 CK_DLL_EXP const char *CK_EXPORT srunner_log_fname(SRunner * sr);
 
 CK_DLL_EXP void CK_EXPORT srunner_set_xml(SRunner * sr, const char *fname);
 
 CK_DLL_EXP int CK_EXPORT srunner_has_xml(SRunner * sr);
 
 CK_DLL_EXP const char *CK_EXPORT srunner_xml_fname(SRunner * sr);
 
 CK_DLL_EXP void CK_EXPORT srunner_set_tap(SRunner * sr, const char *fname);
 
 CK_DLL_EXP int CK_EXPORT srunner_has_tap(SRunner * sr);
 
 CK_DLL_EXP const char *CK_EXPORT srunner_tap_fname(SRunner * sr);
 
 enum fork_status
 {
     CK_FORK_GETENV,             
     CK_FORK,                    
     CK_NOFORK                   
 };
 
 CK_DLL_EXP enum fork_status CK_EXPORT srunner_fork_status(SRunner * sr);
 
 CK_DLL_EXP void CK_EXPORT srunner_set_fork_status(SRunner * sr,
                                                   enum fork_status fstat);
 
 CK_DLL_EXP pid_t CK_EXPORT check_fork(void);
 
 CK_DLL_EXP void CK_EXPORT check_waitpid_and_exit(pid_t pid) CK_ATTRIBUTE_NORETURN;
 
 #ifdef __cplusplus
 CK_CPPEND
 #endif
 
 #endif /* CHECK_H */
