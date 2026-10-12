"""Vectorized PointCloud2 decoding using its declared fields and strides."""
import numpy as np

_TYPES = {1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}


def named_fields(message,names):
    """Zero-copy declared-layout view, retaining rows and nonfinite samples."""
    width,height,step,row = message.width,message.height,message.point_step,message.row_step
    if step<=0 or row<width*step or len(message.data)<height*row:
        raise ValueError('invalid PointCloud2 dimensions or row stride')
    fields = {field.name:field for field in message.fields}
    formats,offsets = [],[]
    for name in names:
        field = fields.get(name)
        if field is None or field.count!=1 or field.datatype not in _TYPES:
            raise ValueError('unsupported PointCloud2 field: '+name)
        dtype = np.dtype(('>' if message.is_bigendian else '<')+_TYPES[field.datatype])
        if field.offset<0 or field.offset+dtype.itemsize>step:
            raise ValueError('PointCloud2 field exceeds point stride')
        formats.append(dtype)
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=list(names),formats=formats,offsets=offsets,itemsize=step))
    return np.ndarray((height,width),dtype=dtype,buffer=message.data,strides=(row,step))


def xyz_intensity(message):
    width,height,step,row = message.width,message.height,message.point_step,message.row_step
    if width==0 or height==0:
        return np.empty((0,4),dtype=np.float32)
    if step<=0 or row<width*step or len(message.data)<height*row:
        raise ValueError('invalid PointCloud2 dimensions or row stride')
    fields = {field.name:field for field in message.fields}
    names = ['x','y','z']+(['intensity'] if 'intensity' in fields else [])
    formats,offsets = [],[]
    endian = '>' if message.is_bigendian else '<'
    for name in names:
        field = fields.get(name)
        if field is None or field.count!=1 or field.datatype not in _TYPES:
            raise ValueError('missing or unsupported PointCloud2 field: '+name)
        if name!='intensity' and field.datatype not in (7,8):
            raise ValueError('PointCloud2 coordinates must be floating point')
        dtype = np.dtype(endian+_TYPES[field.datatype])
        if field.offset<0 or field.offset+dtype.itemsize>step:
            raise ValueError('PointCloud2 field exceeds point stride')
        formats.append(dtype)
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=names,formats=formats,offsets=offsets,itemsize=step))
    view = np.ndarray((height,width),dtype=dtype,buffer=message.data,strides=(row,step))
    result = np.zeros((width*height,4),dtype=np.float32)
    for index,name in enumerate(names):
        result[:,index] = view[name].reshape(-1)
    result = result[np.all(np.isfinite(result[:,:3]),axis=1)]
    result[~np.isfinite(result[:,3]),3] = 0.
    return result
