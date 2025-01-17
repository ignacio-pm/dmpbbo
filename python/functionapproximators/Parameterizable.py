# This file is part of DmpBbo, a set of libraries and programs for the 
# black-box optimization of dynamical movement primitives.
# Copyright (C) 2018 Freek Stulp
# 
# DmpBbo is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
# 
# DmpBbo is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with DmpBbo.  If not, see <http://www.gnu.org/licenses/>.

class Parameterizable:
    
    def setSelectedParameters(selected_values_labels):
        raise NotImplementedError('subclasses must override setSelectedParameters()!')
  
    def getParameterVectorSelected(self):
        raise NotImplementedError('subclasses must override getParameterVectorSelected()!')
    
    def setParameterVectorSelected(self,values):
        raise NotImplementedError('subclasses must override setParameterVectorSelected()!')
        
    def getParameterVectorSelectedSize(self):
        raise NotImplementedError('subclasses must override getParameterVectorSelectedSize()!')
        
        

