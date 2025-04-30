/*
  ************************************************************************\

                               C O P Y R I G H T

    Copyright © 2024 IRMV lab, Shanghai Jiao Tong University, China.
                          All Rights Reserved.

    Licensed under the Creative Commons Attribution-NonCommercial 4.0
    International License (CC BY-NC 4.0).
    You are free to use, copy, modify, and distribute this software and its
    documentation for educational, research, and other non-commercial purposes,
    provided that appropriate credit is given to the original author(s) and
    copyright holder(s).

    For commercial use or licensing inquiries, please contact:
    IRMV lab, Shanghai Jiao Tong University at: https://irmv.sjtu.edu.cn/

                               D I S C L A I M E R

    IN NO EVENT SHALL TRINITY COLLEGE DUBLIN BE LIABLE TO ANY PARTY FOR
    DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING,
    BUT NOT LIMITED TO, LOST PROFITS, ARISING OUT OF THE USE OF THIS SOFTWARE
    AND ITS DOCUMENTATION, EVEN IF TRINITY COLLEGE DUBLIN HAS BEEN ADVISED OF
    THE POSSIBILITY OF SUCH DAMAGES.

    TRINITY COLLEGE DUBLIN DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    PURPOSE. THE SOFTWARE PROVIDED HEREIN IS ON AN "AS IS" BASIS, AND TRINITY
    COLLEGE DUBLIN HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
    ENHANCEMENTS, OR MODIFICATIONS.

    The authors may be contacted at the following e-mail addresses:

            YX.E.Z yixuanzhou@sjtu.edu.cn

    Further information about the IRMV and its projects can be found at the ISG web site :

           https://irmv.sjtu.edu.cn/

  \*************************************************************************
 */

#ifndef DUAL_ARM_APP_FILTER_RAW_HPP
#define DUAL_ARM_APP_FILTER_RAW_HPP
#include "filter_base.hpp"
template<typename T>
class FilterRaw: public FilterBase<T>{
public:
    FilterRaw(): FilterBase<T>(){

    }

    ~FilterRaw() override = default;
public:
    void observe(const T &state) override{

        if(last_data.size() == 0)
            m_data = state;
        else{
            double a = 1.; // full pass
            m_data = state * a + last_data * (1. - a);
        }
        first = false;
    }

    std::optional<T> update() override{
        if(!first){
            if(last_data.size() == 0)
            {
                last_data = m_data;
                return m_data;
            }
            else
            {
                last_data = m_data;
                return m_data;
            }
        }

        return std::nullopt;
    }

private:
    bool first = true;
    T m_data;
    T last_data;
};
#endif //DUAL_ARM_APP_FILTER_RAW_HPP
