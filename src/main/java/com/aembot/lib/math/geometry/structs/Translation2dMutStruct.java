package com.aembot.lib.math.geometry.structs;

import com.aembot.lib.math.geometry.Translation2dMut;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class Translation2dMutStruct implements Struct<Translation2dMut> {
  @Override
  public Class<Translation2dMut> getTypeClass() {
    return Translation2dMut.class;
  }

  @Override
  public String getTypeName() {
    return "Translation2dMut";
  }

  @Override
  public int getSize() {
    return kSizeDouble * 2;
  }

  @Override
  public String getSchema() {
    return "double x;double y";
  }

  @Override
  public Translation2dMut unpack(ByteBuffer bb) {
    double x = bb.getDouble();
    double y = bb.getDouble();
    return new Translation2dMut(x, y);
  }

  @Override
  public void pack(ByteBuffer bb, Translation2dMut value) {
    bb.putDouble(value.getX());
    bb.putDouble(value.getY());
  }

  @Override
  public boolean isImmutable() {
    return false;
  }
}
