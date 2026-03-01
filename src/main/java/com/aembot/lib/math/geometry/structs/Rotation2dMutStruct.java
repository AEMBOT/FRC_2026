package com.aembot.lib.math.geometry.structs;

import com.aembot.lib.math.geometry.Rotation2dMut;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class Rotation2dMutStruct implements Struct<Rotation2dMut> {
  @Override
  public Class<Rotation2dMut> getTypeClass() {
    return Rotation2dMut.class;
  }

  @Override
  public String getTypeName() {
    return "Rotation2dMut";
  }

  @Override
  public int getSize() {
    return kSizeDouble;
  }

  @Override
  public String getSchema() {
    return "double value";
  }

  @Override
  public Rotation2dMut unpack(ByteBuffer bb) {
    double value = bb.getDouble();
    return new Rotation2dMut(value);
  }

  @Override
  public void pack(ByteBuffer bb, Rotation2dMut value) {
    bb.putDouble(value.getRadians());
  }

  @Override
  public boolean isImmutable() {
    return false;
  }
}
