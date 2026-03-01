package com.aembot.lib.math.geometry.structs;

import com.aembot.lib.math.geometry.QuaternionMut;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class QuaternionMutStruct implements Struct<QuaternionMut> {
  @Override
  public Class<QuaternionMut> getTypeClass() {
    return QuaternionMut.class;
  }

  @Override
  public String getTypeName() {
    return "QuaternionMut";
  }

  @Override
  public int getSize() {
    return kSizeDouble * 4;
  }

  @Override
  public String getSchema() {
    return "double w;double x;double y;double z";
  }

  @Override
  public QuaternionMut unpack(ByteBuffer bb) {
    double w = bb.getDouble();
    double x = bb.getDouble();
    double y = bb.getDouble();
    double z = bb.getDouble();
    return new QuaternionMut(w, x, y, z);
  }

  @Override
  public void pack(ByteBuffer bb, QuaternionMut value) {
    bb.putDouble(value.getW());
    bb.putDouble(value.getX());
    bb.putDouble(value.getY());
    bb.putDouble(value.getZ());
  }

  @Override
  public boolean isImmutable() {
    return false;
  }
}
