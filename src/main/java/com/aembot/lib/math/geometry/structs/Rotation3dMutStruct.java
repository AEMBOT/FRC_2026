package com.aembot.lib.math.geometry.structs;

import com.aembot.lib.math.geometry.QuaternionMut;
import com.aembot.lib.math.geometry.Rotation3dMut;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class Rotation3dMutStruct implements Struct<Rotation3dMut> {
  @Override
  public Class<Rotation3dMut> getTypeClass() {
    return Rotation3dMut.class;
  }

  @Override
  public String getTypeName() {
    return "Rotation3dMut";
  }

  @Override
  public int getSize() {
    return Quaternion.struct.getSize();
  }

  @Override
  public String getSchema() {
    return "Quaternion q";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Quaternion.struct};
  }

  @Override
  public Rotation3dMut unpack(ByteBuffer bb) {
    Quaternion q = Quaternion.struct.unpack(bb);
    return new Rotation3dMut(q);
  }

  @Override
  public void pack(ByteBuffer bb, Rotation3dMut value) {
    QuaternionMut.struct.pack(bb, value.getQuaternion());
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}
